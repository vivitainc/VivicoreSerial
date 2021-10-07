#define MIN_LIBRARY_VER_BUILD_NO (0x0012)
#include <VivicoreSerial.h>

#define ANGLE_RANGE   (180)
#define CYCLE_HZ      (50)
#define PULSE_MIN_US  (900)  // the shortest pulse sent to a servo (544 is defined as MIN_PULSE_WIDTH on Servo.h)
#define PULSE_MAX_US  (2100) // the longest pulse sent to a servo (2400 is defined as MAX_PULSE_WIDTH on Servo.h)
#define MAX_CNT       (F_CPU / 8 / CYCLE_HZ) // fclk 8MHz, prescaler 8, freq of servo 50Hz
#define US_PER_CNT    (1000000 / CYCLE_HZ / MAX_CNT)
#define CNT_PER_ANGLE ((PULSE_MAX_US - PULSE_MIN_US) / US_PER_CNT / ANGLE_RANGE)
#define ANGLE_MIN_CNT (PULSE_MIN_US / US_PER_CNT) // equivalent 0 degree
#define ANGLE_MAX_CNT (PULSE_MAX_US / US_PER_CNT) // equivalent 180 degrees
#define DEFAULT_CNT   ((ANGLE_MAX_CNT + ANGLE_MIN_CNT) / 2)
#define CHANNELS      (2)

#define SERVO1_ANGLE_PIN         (10)
#define SERVO2_ANGLE_PIN         (9)
#define BATTERY_ALERT_RESET_PIN  (A1) // LOW active
#define BATTERY_ALERT_DETECT_PIN (A2) // LOW active

#define INTERVAL_MS                      (10)
#define COUNTER_DETECT_EMPTY_BY_OVERLOAD (5)
#define COUNTER_DETECT_EMPTY_BY_INT_IMP  (20)
#define MAX_DURATION_OF_COUNTERS_MS      (60000)
#define MAX_DURATION_OF_A_COUNTER_MS     (15000)
#define COUNTERS                         (MAX_DURATION_OF_COUNTERS_MS / MAX_DURATION_OF_A_COUNTER_MS)
#define COUNTERS_MASK                    (COUNTERS - 1)
#define COUNTERS_FOR_EMPTY_BY_OVERLOAD   (4) // 4 * MAX_DURATION_OF_A_COUNTER_MS = 60s
#define COUNTERS_FOR_EMPTY_BY_INT_IMP    (2) // 2 * MAX_DURATION_OF_A_COUNTER_MS = 30s
#define DURATION_DETECT_ALERT_MS         (60)
#define DURATION_DETECT_EMPTY_MS         (1000)
#define DURATION_DETECT_REPLACING_MS     (1000)
#define DURATION_KEEP_LOW_INDICATION_MS  (15000) // Alert battery low to Core if the state is kept in this duration

enum BatteryState_t {
  BAT_STAT_NORMAL = 0,                   // 0
  BAT_STAT_RECOVERED,                    // 1
  BAT_STAT_WAIT_FOR_STABLE_PIN,          // 2
  BAT_STAT_TRIGGERED_INTERNAL_IMPEDANCE, // 3
  BAT_STAT_WAIT_FOR_EMPTY_TRIGGERED,     // 4
  BAT_STAT_TRIGGERED_OVERLOAD,           // 5
  BAT_STAT_TRIGGERED_EMPTY,              // 6
  BAT_STAT_EMPTY,                        // 7
};

enum BatterySubStateEmpty_t {
  BAT_SUBSTAT_EMPTY_ENTERED = 0,                  // 0
  BAT_SUBSTAT_EMPTY_KEEPING_INDICATOR,            // 1
  BAT_SUBSTAT_EMPTY_RECOVERED,                    // 2
  BAT_SUBSTAT_EMPTY_WAIT_FOR_STABLE_PIN,          // 3
  BAT_SUBSTAT_EMPTY_TRIGGERED_INTERNAL_IMPEDANCE, // 4
  BAT_SUBSTAT_EMPTY_TRIGGERED_OVERLOAD,           // 5
  BAT_SUBSTAT_EMPTY_REPLACING_BATTERY,            // 6
  BAT_SUBSTAT_EMPTY_EXITED,                       // 7
};

class BatteryLowCounter {
public:
  BatteryLowCounter(const uint8_t useCounters, const char *name) : useCounters(useCounters), name(name) {}

  uint8_t increment(const unsigned long curMills) {
    uint16_t sumCount = 0;

    if (curMills - prevMills >= MAX_DURATION_OF_A_COUNTER_MS) {
      if (curMills - prevMills >= MAX_DURATION_OF_COUNTERS_MS) {
        reset();
      } else {
        for (uint8_t i = 0; i < ((curMills - prevMills) / MAX_DURATION_OF_A_COUNTER_MS); i++) {
          buf.index               = (buf.index + 1) & COUNTERS_MASK;
          buf.counters[buf.index] = 0;
        }
      }
      prevMills = curMills;
    }

    if (buf.counters[buf.index] < UINT8_MAX) {
      buf.counters[buf.index] += 1;
    }

    for (uint8_t offset = 0; offset < useCounters; offset++) {
      sumCount += buf.counters[(buf.index - offset) & COUNTERS_MASK];
    }

    return sumCount;
  }

  inline void reset(void) {
    DebugPlainPrint0("reset ");
    DebugPlainPrint0(name);
    DebugPlainPrintln0(" counter");
    memset(&buf, 0, sizeof(buf));
  }

  inline void dumpCounters(void) {
    DebugPlainPrint0(name);
    DebugPlainPrint0(":");
    for (uint8_t offset = 0; offset < useCounters; offset++) {
      DebugPlainPrint0(", count[");
      DebugPlainPrint0((buf.index - offset) & COUNTERS_MASK);
      DebugPlainPrint0("]:");
      DebugPlainPrint0(buf.counters[(buf.index - offset) & COUNTERS_MASK]);
    }
    DebugPlainPrintln0();
  }

private:
  struct CounterBuffer {
    uint8_t index;
    uint8_t counters[COUNTERS];
  };

  const uint8_t useCounters;
  const char *  name;
  unsigned long prevMills = 0;
  CounterBuffer buf       = {};
};

const uint16_t USER_FW_VER = 0x000D;
const uint32_t BRANCH_TYPE = 0x00000001; // Branch index number on vivitainc/ViviParts.git

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max, data_ini}
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_ANALOG_2BYTES, ANGLE_MIN_CNT, ANGLE_MAX_CNT,
   DEFAULT_CNT}, // 1: S1 Angle
  {DcGroup_t::DC_GROUP_2, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_ANALOG_2BYTES, ANGLE_MIN_CNT, ANGLE_MAX_CNT,
   DEFAULT_CNT},                                                                                // 2: S2 Angle
  {DcGroup_t::DC_GROUP_FOR_SYSTEM, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_BOOLEAN, 0, 1}, // 3: Is Battery Low
};

const uint8_t angle_pins[CHANNELS] = {
  SERVO1_ANGLE_PIN,
  SERVO2_ANGLE_PIN,
};

static bool              isBatteryLow = false;
static BatteryLowCounter intImpCounter(COUNTERS_FOR_EMPTY_BY_INT_IMP, "internalImpedance");
static BatteryLowCounter overloadCounter(COUNTERS_FOR_EMPTY_BY_OVERLOAD, "overload");

static inline void analogWriteWithProtection(const int pin, const uint16_t value) {
  // Avoid case to break servo if the value exceeds angle limit
  const uint16_t value_ = constrain(value, ANGLE_MIN_CNT, ANGLE_MAX_CNT);

  if (value_ == 0) {
    digitalWrite(pin, LOW);
  } else if (value_ == MAX_CNT) {
    digitalWrite(pin, HIGH);
  } else {
    if (pin == SERVO1_ANGLE_PIN) {
      bitSet(TCCR1A, COM1B1);
      OCR1B = value_;
    } else if (pin == SERVO2_ANGLE_PIN) {
      bitSet(TCCR1A, COM1A1);
      OCR1A = value_;
    } else {
      // do nothing
    }
  }
}

static inline BatterySubStateEmpty_t getNextEmptyState(const unsigned long          curMills,
                                                       const BatterySubStateEmpty_t nowEmptyState) {
  BatterySubStateEmpty_t nextEmptyState         = nowEmptyState;
  static unsigned long   prevReplaceingMills    = 0;
  static unsigned long   prevUnstableLowMills   = 0;
  static unsigned long   prevLowIndicationMills = 0;

  if (nowEmptyState == BAT_SUBSTAT_EMPTY_ENTERED) {
    prevReplaceingMills    = curMills;
    prevLowIndicationMills = curMills;
    nextEmptyState         = BAT_SUBSTAT_EMPTY_KEEPING_INDICATOR;
  } else if (nowEmptyState == BAT_SUBSTAT_EMPTY_KEEPING_INDICATOR) {
    if (curMills - prevReplaceingMills < DURATION_DETECT_REPLACING_MS) {
      if (isBatteryLow) {
        nextEmptyState = BAT_SUBSTAT_EMPTY_KEEPING_INDICATOR;
      } else {
        nextEmptyState = BAT_SUBSTAT_EMPTY_RECOVERED;
      }
    } else {
      intImpCounter.reset();
      overloadCounter.reset();
      nextEmptyState = BAT_SUBSTAT_EMPTY_REPLACING_BATTERY;
    }
  } else if (nowEmptyState == BAT_SUBSTAT_EMPTY_REPLACING_BATTERY) {
    if (isBatteryLow) {
      nextEmptyState = BAT_SUBSTAT_EMPTY_REPLACING_BATTERY;
    } else {
      nextEmptyState = BAT_SUBSTAT_EMPTY_EXITED;
    }
  } else if (nowEmptyState == BAT_SUBSTAT_EMPTY_RECOVERED) {
    if (curMills - prevLowIndicationMills < DURATION_KEEP_LOW_INDICATION_MS) {
      if (isBatteryLow) {
        prevUnstableLowMills = curMills;
        nextEmptyState       = BAT_SUBSTAT_EMPTY_WAIT_FOR_STABLE_PIN;
      } else {
        nextEmptyState = BAT_SUBSTAT_EMPTY_RECOVERED;
      }
    } else {
      nextEmptyState = BAT_SUBSTAT_EMPTY_EXITED;
    }
  } else if (nowEmptyState == BAT_SUBSTAT_EMPTY_WAIT_FOR_STABLE_PIN) {
    if (curMills - prevUnstableLowMills < DURATION_DETECT_ALERT_MS) {
      if (isBatteryLow) {
        nextEmptyState = BAT_SUBSTAT_EMPTY_WAIT_FOR_STABLE_PIN;
      } else {
        nextEmptyState = BAT_SUBSTAT_EMPTY_TRIGGERED_INTERNAL_IMPEDANCE;
      }
    } else {
      nextEmptyState = BAT_SUBSTAT_EMPTY_TRIGGERED_OVERLOAD;
    }
  } else if (nowEmptyState == BAT_SUBSTAT_EMPTY_TRIGGERED_INTERNAL_IMPEDANCE) {
    if (intImpCounter.increment(curMills) >= COUNTER_DETECT_EMPTY_BY_INT_IMP) {
      intImpCounter.dumpCounters();
      nextEmptyState = BAT_SUBSTAT_EMPTY_ENTERED;
    } else {
      prevReplaceingMills = curMills;
      nextEmptyState      = BAT_SUBSTAT_EMPTY_KEEPING_INDICATOR;
    }
  } else if (nowEmptyState == BAT_SUBSTAT_EMPTY_TRIGGERED_OVERLOAD) {
    if (overloadCounter.increment(curMills) >= COUNTER_DETECT_EMPTY_BY_OVERLOAD) {
      overloadCounter.dumpCounters();
      nextEmptyState = BAT_SUBSTAT_EMPTY_ENTERED;
    } else {
      prevReplaceingMills = curMills;
      nextEmptyState      = BAT_SUBSTAT_EMPTY_KEEPING_INDICATOR;
    }
  }

  return nextEmptyState;
}

static inline BatteryState_t getNextState(const unsigned long curMills, const BatteryState_t nowState) {
  BatteryState_t                nextState       = nowState;
  static BatterySubStateEmpty_t nowEmptyState   = BAT_SUBSTAT_EMPTY_EXITED;
  BatterySubStateEmpty_t        nextEmptyState  = nowEmptyState;
  static unsigned long          prevStatusMills = 0;

  isBatteryLow = (digitalRead(BATTERY_ALERT_DETECT_PIN) == LOW);
  if (isBatteryLow) {
    digitalWrite(BATTERY_ALERT_RESET_PIN, LOW);
    digitalWrite(BATTERY_ALERT_RESET_PIN, HIGH);
  }

  if (nowState == BAT_STAT_NORMAL) {
    if (isBatteryLow) {
      prevStatusMills = curMills;
      nextState       = BAT_STAT_WAIT_FOR_STABLE_PIN;
    } else {
      nextState = BAT_STAT_NORMAL;
    }
  } else if (nowState == BAT_STAT_WAIT_FOR_STABLE_PIN) {
    if (curMills - prevStatusMills < DURATION_DETECT_ALERT_MS) {
      if (isBatteryLow) {
        nextState = BAT_STAT_WAIT_FOR_STABLE_PIN;
      } else {
        nextState = BAT_STAT_TRIGGERED_INTERNAL_IMPEDANCE;
      }
    } else {
      prevStatusMills = curMills;
      nextState       = BAT_STAT_WAIT_FOR_EMPTY_TRIGGERED;
    }
  } else if (nowState == BAT_STAT_WAIT_FOR_EMPTY_TRIGGERED) {
    if (curMills - prevStatusMills < DURATION_DETECT_EMPTY_MS) {
      if (isBatteryLow) {
        nextState = BAT_STAT_WAIT_FOR_EMPTY_TRIGGERED;
      } else {
        nextState = BAT_STAT_TRIGGERED_OVERLOAD;
      }
    } else {
      nextState = BAT_STAT_TRIGGERED_EMPTY;
    }
  } else if (nowState == BAT_STAT_TRIGGERED_INTERNAL_IMPEDANCE) {
    if (intImpCounter.increment(curMills) >= COUNTER_DETECT_EMPTY_BY_INT_IMP) {
      intImpCounter.dumpCounters();
      nextState = BAT_STAT_TRIGGERED_EMPTY;
    } else {
      nextState = BAT_STAT_NORMAL;
    }
  } else if (nowState == BAT_STAT_TRIGGERED_OVERLOAD) {
    if (overloadCounter.increment(curMills) >= COUNTER_DETECT_EMPTY_BY_OVERLOAD) {
      overloadCounter.dumpCounters();
      nextState = BAT_STAT_TRIGGERED_EMPTY;
    } else {
      nextState = BAT_STAT_NORMAL;
    }
  } else if (nowState == BAT_STAT_TRIGGERED_EMPTY) {
    nowEmptyState = BAT_SUBSTAT_EMPTY_ENTERED;
    nextState     = BAT_STAT_EMPTY;
  } else if (nowState == BAT_STAT_EMPTY) {
    nextEmptyState = getNextEmptyState(curMills, nowEmptyState);
    if (nowEmptyState != nextEmptyState) {
      if (nextEmptyState == BAT_SUBSTAT_EMPTY_EXITED) {
        nextState = BAT_STAT_RECOVERED;
      } else if (nextEmptyState == BAT_SUBSTAT_EMPTY_TRIGGERED_INTERNAL_IMPEDANCE) {
        DebugPlainPrintln0("!");
      } else if (nextEmptyState == BAT_SUBSTAT_EMPTY_TRIGGERED_OVERLOAD) {
        DebugPlainPrintln0("*");
      }
      DebugPlainPrint2("(sub) now:");
      DebugPlainPrint2(nowEmptyState);
      DebugPlainPrint2(", low:");
      DebugPlainPrint2(isBatteryLow);
      DebugPlainPrint2(", next:");
      DebugPlainPrintln2(nextEmptyState);
    }
    nowEmptyState = nextEmptyState;
  } else if (nowState == BAT_STAT_RECOVERED) {
    nextState = BAT_STAT_NORMAL;
  }

  return nextState;
}

static inline void detectLowBattery(const unsigned long curMills) {
  static BatteryState_t now       = BAT_STAT_NORMAL;
  BatteryState_t        next      = now;
  static unsigned long  prevMills = 0;

  if (curMills - prevMills < INTERVAL_MS) {
    return;
  }
  prevMills = curMills;

  next = getNextState(curMills, now);
  if (now == next) {
    return;
  }

  DebugPlainPrint2("now:");
  DebugPlainPrint2(now);
  DebugPlainPrint2(", low:");
  DebugPlainPrint2(isBatteryLow);
  DebugPlainPrint2(", next:");
  DebugPlainPrintln2(next);

  if (next == BAT_STAT_RECOVERED) {
    Vivicore.write(3, false); // 3: Is Battery Low
    DebugPlainPrintln0("Send battery normal");
  } else if (next == BAT_STAT_TRIGGERED_EMPTY) {
    Vivicore.write(3, true); // 3: Is Battery Low
    DebugPlainPrintln0("Low and send battery empty");
  } else if (next == BAT_STAT_TRIGGERED_INTERNAL_IMPEDANCE) {
    DebugPlainPrintln0("!");
  } else if (next == BAT_STAT_TRIGGERED_OVERLOAD) {
    DebugPlainPrintln0("*");
  } else {
    // do nothing
  }
  Vivicore.flush();

  now = next;
}

static inline void driveServo(const unsigned long curMills) {
  static uint16_t      prev_angles[CHANNELS] = {}; // Initialize as count 0 to apply the first received data from app
  AvailableNum_t       cnt                   = {};
  static unsigned long prevMills             = 0;

  if (curMills - prevMills < INTERVAL_MS) {
    return;
  }
  prevMills = curMills;

  cnt = Vivicore.available();
  for (uint8_t i = 0; i < cnt.scaler; i++) {
    const ScalerData_t scaler = Vivicore.read();
    const uint16_t     angle  = static_cast<uint16_t>(scaler.data);

    if (scaler.success && (0 < scaler.dc_n) && (scaler.dc_n <= CHANNELS)) {
      const uint8_t ch = scaler.dc_n - 1;
      if (angle != prev_angles[ch]) {
        prev_angles[ch] = angle;
        analogWriteWithProtection(angle_pins[ch], angle);
      }
    }

    DebugPlainPrint0("success:");
    DebugPlainPrint0(scaler.success);
    DebugPlainPrint0(", angle");
    DebugPlainPrint0(scaler.dc_n);
    DebugPlainPrint0(":");
    DebugPlainPrintln0(angle);
  }
}

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);

  digitalWrite(BATTERY_ALERT_RESET_PIN, HIGH);
  pinMode(BATTERY_ALERT_RESET_PIN, OUTPUT);
  pinMode(BATTERY_ALERT_DETECT_PIN, INPUT_PULLUP);
  pinMode(SERVO1_ANGLE_PIN, OUTPUT);
  pinMode(SERVO2_ANGLE_PIN, OUTPUT);

  // Activate RESET# at once for alert circuit to work
  digitalWrite(BATTERY_ALERT_RESET_PIN, LOW);
  digitalWrite(BATTERY_ALERT_RESET_PIN, HIGH);

  TCCR1A = bit(COM1A1) | bit(COM1B1) | // no inverting
           bit(WGM11);
  TCCR1B = bit(WGM13) | bit(WGM12) | // fast PWM, TOP=ICR1
           bit(CS11);                // clk/8 prescaler
  ICR1 = MAX_CNT;                    // TOP counter value

  for (int ch = 0; ch < CHANNELS; ch++) {
    // Do not apply torque to avoid moving to DEFAULT_CNT at the time of app's initializing hardware module
    digitalWrite(angle_pins[ch], LOW);
  }
}

void loop() {
  const unsigned long curMills = millis();
  detectLowBattery(curMills);
  driveServo(curMills);
}
