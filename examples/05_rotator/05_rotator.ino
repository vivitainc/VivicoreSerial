#define MIN_LIBRARY_VER_BUILD_NO (0x0012)
#include <VivicoreSerial.h>

// #define CHATTERING_WORKAROUND // Enable workaround to exclude chattering
// #define CHECK_PULSE_MISS_DETECTION // Enable check if pulse detection miss happens

#define HW_PIN_PULSE_A (A0)    // Hardware spec for FP1
#define HW_PIN_PULSE_B (A2)    // Hardware spec for FP1
#define HW_PORT_PULSES (PINC)  // Hardware spec for FP1 (PulseA & PulseB)
#define HW_BIT_PULSE_A (PINC0) // Hardware spec for FP1 (PulseA = A0 PC0)
#define HW_BIT_PULSE_B (PINC2) // Hardware spec for FP1 (PulseB = A2 PC2)
#define HW_PIN_SW      (2)     // Hardware spec for FP1

#define BIT_PULSE_A         (B00000010)
#define BIT_PULSE_B         (B00000001)
#define BITS_PULSE_A_B      (BIT_PULSE_A | BIT_PULSE_B)
#define CLICK_PULSE         (BIT_PULSE_B) // BIT_PULSE_A or BIT_PULSE_B or BITS_PULSE_A_B to update countVal
#define BITS_PRE_CUR_PULSES (B00001111)

#define CNT_FIX_INTERVAL_MS (30)
#define ABS_MAX             (100)
#define MIN_VALUE           ((-1) * (ABS_MAX))
#define MAX_VALUE           (ABS_MAX)
#define INITIAL_VAL         (0)

#define DEFAULT_RESET (0)
#define DEFAULT_WRAP  (1)

#define TIMER_TRIGGER_US    (256L) // (1024L)
#define TIMER_TRIGGER_COUNT (TIMER_TRIGGER_US * ((F_CPU / 1L) / 1000L / 1000L))

const uint8_t  USER_FW_MAJOR_VER = 0x01;
const uint8_t  USER_FW_MINOR_VER = 0x02;
const uint16_t USER_FW_VER       = (((uint16_t)(USER_FW_MAJOR_VER) << 8) + ((uint16_t)(USER_FW_MINOR_VER)));
const uint32_t BRANCH_TYPE       = 0x00000005; // Branch index number on vivitainc/ViviParts.git

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_BOOLEAN, 0, 1}, // 1: Button
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_1BYTE, MIN_VALUE, MAX_VALUE,
   INITIAL_VAL},                                                                                     // 2: Value
  {DcGroup_t::DC_GROUP_2, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_BOOLEAN, 0, 1, DEFAULT_RESET}, // 3: Reset
  {DcGroup_t::DC_GROUP_2, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_BOOLEAN, 0, 1, DEFAULT_WRAP},  // 4: Wrap
};

static signed int          encTable[0x10] = {};
static volatile signed int countVal       = INITIAL_VAL;
static volatile bool       wrapEnabled    = (bool)DEFAULT_WRAP;

typedef enum {
  iButton = 1,
  iValue,
  iReset,
  iWrap,
} dcInfoIndex_t;

#if defined(CHECK_PULSE_MISS_DETECTION)
struct RotateEle_t {
  uint8_t right;
  uint8_t left;
};

RotateEle_t rotate[B100] = {};
#endif

static inline void setEncTable(void) {
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

#if defined(CHECK_PULSE_MISS_DETECTION)
  // Clockwise
  rotate[B00].right = B10;
  rotate[B10].right = B11;
  rotate[B11].right = B01;
  rotate[B01].right = B00;
  // Counter-clockwise
  rotate[B00].left = B01;
  rotate[B10].left = B00;
  rotate[B11].left = B10;
  rotate[B01].left = B11;
#endif
}

static inline signed int adjustForClick(const signed int curDelta, const uint8_t encVal) {
  signed int     adjustedDelta = 0;
  const uint8_t  curPulse      = encVal & CLICK_PULSE;
  static uint8_t prePulse      = curPulse;

  if (prePulse != curPulse) {
    prePulse      = curPulse;
    adjustedDelta = curDelta;
  }

  return adjustedDelta;
}

static inline uint8_t getEncVal(void) {
  static uint8_t encVal = 0;

  // cli(); // no needed as this function is called in interrupt
  const uint8_t regPortC = HW_PORT_PULSES;
  const uint8_t pulseA   = bitRead(regPortC, HW_BIT_PULSE_A); // digitalRead(HW_PIN_PULSE_A);
  const uint8_t pulseB   = bitRead(regPortC, HW_BIT_PULSE_B); // digitalRead(HW_PIN_PULSE_B);
  // sei(); // no needed as this function is called in interrupt

  const uint8_t curEncVal = (pulseA << 1) + pulseB;
  encVal                  = (encVal << 2) + curEncVal;

  return encVal;
}

static inline signed int fixCount(const signed int val) {
  signed int fixVal = val;
  if (val < MIN_VALUE || val > MAX_VALUE) {
    if (wrapEnabled) {
      if (val < MIN_VALUE) {
        fixVal = MAX_VALUE - (abs(val - MIN_VALUE) - 1);
      } else if (val > MAX_VALUE) {
        fixVal = MIN_VALUE + (abs(val - MAX_VALUE) - 1);
      }
    } else {
      fixVal = constrain(val, MIN_VALUE, MAX_VALUE);
    }
  }
  return fixVal;
}

static inline void processEncVal(void) {
  const uint8_t    encVal   = getEncVal();
  const signed int curDelta = encTable[encVal & BITS_PRE_CUR_PULSES];

  static uint8_t preEncVal = 0;
  const uint8_t  prePulses = preEncVal & BITS_PULSE_A_B;
  const uint8_t  curPulses = encVal & BITS_PULSE_A_B;

  if (prePulses == curPulses) {
    // no need to update countVal if no pulse state change
    return;
  }

#if defined(CHECK_PULSE_MISS_DETECTION)
  if (rotate[prePulses].left != curPulses && rotate[prePulses].right != curPulses) {
    // miss detection of a pulse edge
    DebugPlainPrintln0("+");
    DebugGPIOHigh(PORTC, 4); // debug A4 PC4
    DebugGPIOLow(PORTC, 4);  // debug A4 PC4
  }
#endif

  preEncVal = encVal;

  if (curDelta != 0) {
#if defined(CHATTERING_WORKAROUND)
    // Workaround to exclude chattering
    if (curDelta * (-1) == encTable[(encVal >> 2) & BITS_PRE_CUR_PULSES]) {
      return;
    }
#endif
    if (curDelta > 0) {
      DebugBinPrint0(encVal);
      DebugPlainPrintln0(",CW");
    } else if (curDelta < 0) {
      DebugBinPrint0(encVal);
      DebugPlainPrintln0(",CCW");
    }
    const signed int delta = adjustForClick(curDelta, encVal);
    countVal += delta;
    countVal = fixCount(countVal);
  }
}

ISR(TIMER4_COMPA_vect) {
  DebugGPIOHigh(PORTC, 5); // debug A5 PC5
  processEncVal();
  DebugGPIOLow(PORTC, 5); // debug A5 PC5
}

static bool updateButton(void) {
  static uint8_t prevButtonState = 0;
  const uint8_t  curButtonState  = !digitalRead(HW_PIN_SW);
  bool           willSend        = false;

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

static void sendToCore(const signed int curVal, const bool resetOn, const bool wrapEnabled) {
  static int8_t prevSndVal = INITIAL_VAL;
  const int8_t  sndVal     = (int8_t)curVal;

  bool willSend = updateButton();

  if (prevSndVal != sndVal || resetOn) {
    DebugPlainPrint0("wrap=");
    DebugPlainPrint0(wrapEnabled);
    DebugPlainPrint0(",");
    DebugPlainPrint0("rst=");
    DebugPlainPrint0(resetOn);
    DebugPlainPrint0(",");
    DebugPlainPrint0("sndVal=");
    DebugPlainPrint0(sndVal);
    DebugPlainPrint0(",");

    prevSndVal = sndVal;
    Vivicore.write(iValue, sndVal);
    willSend = true;
  }

  if (willSend) {
    Vivicore.flush();
    DebugPlainPrint0("snt");
    DebugPlainPrintln0("");
  }
}

static bool readFromCore(void) {
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

static void startTimer4Interrupt() {
  // Timer 4 settings
  TCCR4A = 0;
  TCCR4B = bit(WGM42) | bit(CS40); // CTC mode, No prescaling
  OCR4A  = TIMER_TRIGGER_COUNT;    // ISR trigger counter value
  TIMSK4 = bit(OCIE4A);            // Output Compare A Match Interrupt Enable
}

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);

  pinMode(HW_PIN_PULSE_A, INPUT);
  pinMode(HW_PIN_PULSE_B, INPUT);
  pinMode(HW_PIN_SW, INPUT);

  setEncTable();

  DebugGPIODirectOut(DDRC, 5); // debug A5 PC5
  DebugGPIOLow(PORTC, 5);      // debug A5 PC5
  DebugGPIODirectOut(DDRC, 4); // debug A4 PC4
  DebugGPIOLow(PORTC, 4);      // debug A4 PC4

  startTimer4Interrupt();
}

void loop() {
  static unsigned long prevMills = 0;
  const unsigned long  curMills  = millis();

  // Dont care about millis overflows https://garretlab.web.fc2.com/arduino/lab/millis/
  if (curMills - prevMills > CNT_FIX_INTERVAL_MS) {
    prevMills = curMills;

    const bool resetOn = readFromCore();

    cli();
    if (resetOn) {
      countVal = INITIAL_VAL;
    }
    const signed int curVal = countVal;
    sei();

    sendToCore(curVal, resetOn, wrapEnabled);
  }
}
