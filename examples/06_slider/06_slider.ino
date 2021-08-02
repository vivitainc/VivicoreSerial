#define MIN_LIBRARY_VER_BUILD_NO (0x0012)
#include <VivicoreSerial.h>

#define AVG_TIME             (10)
#define ACCEPTABLE_ADC_ERROR (5)
#define AD_MIN_VALUE         (0)
#define AD_MAX_VALUE         (1023)
#define DAT_MIN_VALUE        (AD_MIN_VALUE + ACCEPTABLE_ADC_ERROR - 1) // 4
#define DAT_MAX_VALUE        (AD_MAX_VALUE - ACCEPTABLE_ADC_ERROR + 1) // 1019

#define AD_PRESCALER_2   (bit(ADPS0))
#define AD_PRESCALER_4   (bit(ADPS1))
#define AD_PRESCALER_8   (bit(ADPS1) | bit(ADPS0))
#define AD_PRESCALER_16  (bit(ADPS2))
#define AD_PRESCALER_32  (bit(ADPS2) | bit(ADPS0))
#define AD_PRESCALER_64  (bit(ADPS2) | bit(ADPS1))
#define AD_PRESCALER_128 (bit(ADPS2) | bit(ADPS1) | bit(ADPS0))

const uint16_t USER_FW_VER = 0x0008;
const uint32_t BRANCH_TYPE = 0x00000006;

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_2BYTES, DAT_MIN_VALUE,
   DAT_MAX_VALUE}, // 1: Value
};

typedef struct {
  double  rawVal;
  int16_t intVal;
  int16_t trimVal;
} valueSet_t;

static valueSet_t prevSet = {};

static int16_t trimEdgeValue(const int16_t val, const int16_t min, const int16_t max, const int16_t err,
                             const int16_t dataMin, const int16_t dataMax) {
  int16_t trimVal = val;
  if (trimVal < err) {
    trimVal = dataMin;
  } else if (trimVal > (max - err)) {
    trimVal = dataMax;
  }
  return trimVal;
}

static valueSet_t getADCalcValue(void) {
  valueSet_t valSet = {};
  int16_t    sum    = 0;

  for (int i = 0; i < AVG_TIME; i++) {
    sum += (int16_t)analogRead(A0);
  }

  valSet.rawVal = (double)sum / AVG_TIME;
  valSet.intVal = (int16_t)roundf(valSet.rawVal);

  valSet.trimVal =
    trimEdgeValue(valSet.intVal, AD_MIN_VALUE, AD_MAX_VALUE, ACCEPTABLE_ADC_ERROR, DAT_MIN_VALUE, DAT_MAX_VALUE);

  return valSet;
}

void setup() {
  ADCSRA |= AD_PRESCALER_128;

  prevSet = getADCalcValue();

  Vivicore.setOverrideIni(1, prevSet.trimVal, dcInfo, countof(dcInfo));
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);

  DebugPlainPrint0("ADCSRA: ");
  DebugPlainPrint0(ADCSRA, BIN);
  DebugPlainPrintln0();
}

void loop() {
  boolean snd_flg = false;

  delay(10);

  const valueSet_t curSet = getADCalcValue();

  DebugPlainPrint0(prevSet.rawVal);
  DebugPlainPrint0(", ");
  DebugPlainPrint0(curSet.rawVal);
  DebugPlainPrint0(", ");
  DebugPlainPrint0(curSet.intVal);
  DebugPlainPrint0(", ");
  DebugPlainPrint0(curSet.trimVal);
  DebugPlainPrint0(", ");

  if (curSet.trimVal != prevSet.trimVal) {
    if (abs(curSet.rawVal - prevSet.rawVal) > ACCEPTABLE_ADC_ERROR) {
      snd_flg = true;
    } else if ((curSet.trimVal == DAT_MIN_VALUE) || (curSet.trimVal == DAT_MAX_VALUE)) {
      snd_flg = true;
    }
  }

  DebugPlainPrint0(snd_flg);
  DebugPlainPrintln0();

  if (snd_flg) {
    Vivicore.write(1, curSet.trimVal); // 1: Value
    prevSet = curSet;
  } else if ((curSet.trimVal == DAT_MIN_VALUE) && (prevSet.rawVal > curSet.rawVal)) {
    prevSet.rawVal = curSet.rawVal;
  } else if ((curSet.trimVal == DAT_MAX_VALUE) && (prevSet.rawVal < curSet.rawVal)) {
    prevSet.rawVal = curSet.rawVal;
  }

  Vivicore.flush();
}
