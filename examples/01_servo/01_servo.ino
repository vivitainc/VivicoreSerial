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

#define LOOP_INTERVAL        (10)   // Unit: ms.
#define DURATION_BATTERY_LOW (1000) // Unit: ms. Alert battery low to Core if the state is kept in this duration

const uint16_t USER_FW_VER = 0x000B;
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

static inline void detectLowBattery(void) {
  static uint16_t batteryLowDuration = 0;
  static bool     prevBatteryLow     = false;
  const bool      isBatteryLow       = (digitalRead(BATTERY_ALERT_DETECT_PIN) == LOW);

  if (isBatteryLow) {
    batteryLowDuration += LOOP_INTERVAL;
    digitalWrite(BATTERY_ALERT_RESET_PIN, LOW);
    digitalWrite(BATTERY_ALERT_RESET_PIN, HIGH);
  } else {
    batteryLowDuration = 0;
  }

  if (prevBatteryLow != isBatteryLow) {
    bool shouldNotify = false;
    if (isBatteryLow) {
      if (batteryLowDuration > DURATION_BATTERY_LOW) {
        shouldNotify = true;
      }
    } else {
      shouldNotify = true;
    }

    if (shouldNotify) {
      prevBatteryLow = isBatteryLow;
      Vivicore.write(3, isBatteryLow); // 3: Is Battery Low
    }

    Vivicore.flush();

    DebugPlainPrint0("duration:");
    DebugPlainPrint0(batteryLowDuration);
    DebugPlainPrint0(", low:");
    DebugPlainPrintln0(isBatteryLow);
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
    analogWriteWithProtection(angle_pins[ch], DEFAULT_CNT);
  }
}

void loop() {
  static uint16_t prev_angles[CHANNELS] = {
    DEFAULT_CNT,
    DEFAULT_CNT,
  };
  const AvailableNum_t cnt = Vivicore.available();

  delay(LOOP_INTERVAL);

  detectLowBattery();

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
