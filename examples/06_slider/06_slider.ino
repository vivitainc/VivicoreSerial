#define MIN_LIBRARY_VER_BUILD_NO (0x0004)
#include <VivicoreSerial.h>

#define AVG_TIME (10)
#define ACCEPTABLE_ADC_ERROR (5)
#define MIN_VALUE (0)
#define MAX_VALUE (1023)

#define AD_RESOLUTION    (10)
#define AD_PRESCALER_2   (bit(ADPS0))
#define AD_PRESCALER_4   (bit(ADPS1))
#define AD_PRESCALER_8   (bit(ADPS1) | bit(ADPS0))
#define AD_PRESCALER_16  (bit(ADPS2))
#define AD_PRESCALER_32  (bit(ADPS2) | bit(ADPS0))
#define AD_PRESCALER_64  (bit(ADPS2) | bit(ADPS1))
#define AD_PRESCALER_128 (bit(ADPS2) | bit(ADPS1) | bit(ADPS0))

const uint16_t USER_FW_VER = 0x0005;
const uint32_t BRANCH_TYPE = 0x00000006;

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DC_GROUP_1, DC_NATURE_OUT, DC_TYPE_ANALOG_2BYTES, MIN_VALUE, MAX_VALUE},  // 1: Value
};
uint8_t stateBuffer[] = {
  1, 0, 0,  // 1: Value
};

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);

  ADCSRA |= AD_PRESCALER_128;
  DebugPlainPrint0("ADCSRA: ");
  DebugPlainPrint0(ADCSRA, BIN);
  DebugPlainPrintln0();
}

void loop() {
  static uint16_t prevVal = 0;
  static double prevRawVal = 0;
  uint16_t sum  = 0;
  boolean snd_flg = false;
  uint16_t val = 0;
  double rawVal = 0.0;

  delay(10);

  for(int i = 0; i < AVG_TIME; i++){
    sum += (uint16_t)analogRead(A0);
  }

  rawVal = (double)sum / AVG_TIME;

  if (abs(rawVal - prevRawVal) > ACCEPTABLE_ADC_ERROR) {
    prevRawVal = rawVal;
    snd_flg = true;
  }

  DebugPlainPrint0(rawVal);
  DebugPlainPrint0(", ");

  val = ((rawVal * ((MAX_VALUE - MIN_VALUE) + 1)) / pow(2, AD_RESOLUTION));

  DebugPlainPrint0(val);
  DebugPlainPrint0(", ");
  DebugPlainPrint0(snd_flg);
  DebugPlainPrintln0();

  if (snd_flg && (val != prevVal)) {
    uint8_t offset = 1;
    stateBuffer[offset++] = highByte(val);
    stateBuffer[offset++] = lowByte(val);
    Vivicore.write(stateBuffer, sizeof(stateBuffer));
    prevVal = val;
  }

  Vivicore.flush();
}
