#define MIN_LIBRARY_VER_BUILD_NO (0x0012)
#include <VivicoreSerial.h>

//#define DUMMY_DELTA        // For debug purpose only

#define CHATTERING_WORKAROUND // Enable workaround to exclude chattering

#define PIN_PULSE_A (A0) // Hardware spec for FP1
#define PIN_PULSE_B (A2) // Hardware spec for FP1
#define PIN_SW      (2)  // Hardware spec for FP1

#define CNT_FIX_INTERVAL_MS (30)
#define MIN_VALUE           (0)
#define MAX_VALUE           (100)
#define DELTA_MAX           (MAX_VALUE / 2)
#define DELTA_MIN           ((-1) * (DELTA_MAX))
#define INITIAL_VAL         (DELTA_MIN)

#define DEFAULT_RESET (0)
#define DEFAULT_WRAP  (0)

const uint16_t USER_FW_VER = 0x000B;
const uint32_t BRANCH_TYPE = 0x00000005; // Branch index number on vivitainc/ViviParts.git

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_BOOLEAN, 0, 1},                      // 1: Button
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_1BYTE, MIN_VALUE, MAX_VALUE}, // 2: Value
  {DcGroup_t::DC_GROUP_2, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_BOOLEAN, 0, 1, DEFAULT_RESET},        // 3: Reset
  {DcGroup_t::DC_GROUP_2, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_BOOLEAN, 0, 1, DEFAULT_WRAP},         // 4: Wrap
};

static signed int encTable[0x10] = {};

typedef enum {
  iButton = 1,
  iValue,
  iReset,
  iWrap,
} dcInfoIndex_t;

static void setEncTable(void) {
  // Clockwise
  encTable[B00000010] = +1;
  encTable[B00001011] = +1;
  encTable[B00001101] = +1;
  encTable[B00000100] = +1;
  // Counter-clockwise
  encTable[B00000001] = -1;
  encTable[B00000111] = -1;
  encTable[B00001110] = -1;
  encTable[B00001000] = -1;
}

static uint8_t getEncVal(void) {
  static uint8_t encVal = 0;

  cli();
  uint8_t pulseA = digitalRead(PIN_PULSE_A);
  uint8_t pulseB = digitalRead(PIN_PULSE_B);
  sei();

  encVal = (encVal << 2) + ((pulseA << 1) + pulseB);
  return encVal;
}

static signed int processEncVal(const uint8_t encVal) {
  signed int delta = 0;
#if defined(CHATTERING_WORKAROUND)
  signed int curDelta = encTable[encVal & B00001111];
  if (curDelta != 0) {
    // Workaround to exclude chattering
    if (curDelta * (-1) != encTable[(encVal >> 2) & B00001111]) {
      delta = curDelta;
      if (delta > 0) {
        DebugBinPrint0(encVal);
        DebugPlainPrintln0(",CW");
      } else if (delta < 0) {
        DebugBinPrint0(encVal);
        DebugPlainPrintln0(",CCW");
      }
    } else {
      // DebugBinPrint0(encVal);
      // DebugPlainPrintln0(",None");
    }
  }
#else  // CHATTERING_WORKAROUND
  switch (encVal & B00001111) {
  case B00000010:
  case B00001011:
  case B00001101:
  case B00000100:
    delta++;
    DebugBinPrint0(encVal);
    DebugPlainPrintln0(",CW");
    break;
  case B00000001:
  case B00000111:
  case B00001110:
  case B00001000:
    delta--;
    DebugBinPrint0(encVal);
    DebugPlainPrintln0(",CCW");
    break;
  default:
    // DebugBinPrint0(encVal);
    // DebugPlainPrintln0(",None");
    break;
  }
#endif // CHATTERING_WORKAROUND
  return delta;
}

static signed int adjustDelta(const signed int curDelta) {
  static signed int prevDelta     = 0;
  signed int        adjustedDelta = 0;

  if (curDelta != 0) {
    if (prevDelta == curDelta) {
      adjustedDelta = curDelta;
      prevDelta     = 0;
    } else {
      prevDelta = curDelta;
    }
  }

  return adjustedDelta;
}

static bool updateButton(void) {
  static uint8_t prevButtonState = 0;
  uint8_t        curButtonState  = 0;
  bool           willSend        = false;

  curButtonState = !digitalRead(PIN_SW);
  if (curButtonState != prevButtonState) {
    DebugPlainPrint0("btn:");
    DebugPlainPrint0(curButtonState);
    DebugPlainPrint0(",");

    prevButtonState = curButtonState;
    Vivicore.write(iButton, curButtonState);
    willSend = true;
  }

  return willSend;
}

static void sendToCore(signed int delta, const bool resetOn, const bool wrapEnabled) {
  static signed int keptVal    = INITIAL_VAL;
  static uint8_t    prevSndVal = map(INITIAL_VAL, DELTA_MIN, DELTA_MAX, MIN_VALUE, MAX_VALUE);

  bool willSend = updateButton();

  if (delta != 0 || resetOn) {
    uint8_t sndVal = 0;
    if (resetOn) {
      keptVal = INITIAL_VAL;
    }

    DebugPlainPrint0("rst:");
    DebugPlainPrint0(resetOn);
    DebugPlainPrint0(",");

    DebugPlainPrint0("delta:");
    DebugPlainPrint0(delta);
    DebugPlainPrint0(",");

    delta = constrain(delta, DELTA_MIN, DELTA_MAX);

    keptVal += delta;
    DebugPlainPrint0("keptVal:");
    DebugPlainPrint0(keptVal);
    DebugPlainPrint0(",");
    DebugPlainPrint0("wrap=");
    DebugPlainPrint0(wrapEnabled);
    DebugPlainPrint0(":");

    if (abs(keptVal) > DELTA_MAX) {
      if (wrapEnabled) {
        const signed int sign           = (keptVal < 0) ? -1 : +1;
        const signed int signedLimitVal = sign * DELTA_MAX;
        const signed int outlyingVal    = (abs(keptVal) % DELTA_MAX - 1) * sign;
        keptVal                         = (-1) * signedLimitVal + outlyingVal;
      } else {
        keptVal = constrain(keptVal, DELTA_MIN, DELTA_MAX);
      }
    }
    DebugPlainPrint0(keptVal);
    DebugPlainPrint0(",");

    sndVal = map(keptVal, DELTA_MIN, DELTA_MAX, MIN_VALUE, MAX_VALUE);
    DebugPlainPrint0("sndVal:");
    DebugPlainPrint0(sndVal);
    DebugPlainPrint0(",");

    if (prevSndVal != sndVal || resetOn) {
      prevSndVal = sndVal;
      Vivicore.write(iValue, sndVal);
      willSend = true;
    }
  }

  if (willSend) {
    Vivicore.flush();
    DebugPlainPrint0("snt");
    DebugPlainPrintln0("");
  }
}

static bool readFromCore(bool &wrapEnabled) {
  const AvailableNum_t cnt     = Vivicore.available();
  bool                 resetOn = (bool)DEFAULT_RESET;

  for (uint8_t i = 0; i < cnt.scaler; i++) {
    const ScalerData_t scaler = Vivicore.read();
    if (scaler.success) {
      if (scaler.dc_n == iReset) {
        resetOn = (bool)scaler.data;
      } else if (scaler.dc_n == iWrap) {
        wrapEnabled = (bool)scaler.data;
      }
    }
    DebugPlainPrint0("dc_n:");
    DebugPlainPrint0(scaler.dc_n);
    DebugPlainPrint0(", success:");
    DebugPlainPrint0(scaler.success);
    DebugPlainPrint0(", data:");
    DebugPlainPrintln0(scaler.data);
  }

  return resetOn;
}

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);

  pinMode(PIN_PULSE_A, INPUT);
  pinMode(PIN_PULSE_B, INPUT);
  pinMode(PIN_SW, INPUT);

  setEncTable();
}

void loop() {
  static signed int    delta        = 0;
  static bool          wrapEnabled  = (bool)DEFAULT_WRAP;
  static unsigned long prevMills    = 0;
  unsigned long        curMills     = millis();
  uint8_t              encVal       = 0;
  signed int           instantDelta = 0;

  encVal       = getEncVal();
  instantDelta = processEncVal(encVal);
  delta += adjustDelta(instantDelta);

  // Dont care about millis overflows https://garretlab.web.fc2.com/arduino/lab/millis/
  if (curMills - prevMills > CNT_FIX_INTERVAL_MS) {
    prevMills = curMills;
#if defined(DUMMY_COUNTER)
    static signed int snd_cnt = 0;
    static signed int step    = 1;
    delta                     = snd_cnt;
    snd_cnt += step;
    if (snd_cnt > MAX_VALUE || snd_cnt < MIN_VALUE) {
      step *= -1;
      snd_cnt += (step * 2);
    }
#endif // DUMMY_COUNTER

    bool resetOn = readFromCore(wrapEnabled);
    sendToCore(delta, resetOn, wrapEnabled);
    delta = 0;
  }
}
