#define MIN_LIBRARY_VER_BUILD_NO (0x0012)
#include <VivicoreSerial.h>

#define BATTERY_ALERT_RESET_PIN  (A1) // LOW active
#define BATTERY_ALERT_DETECT_PIN (A2) // LOW active
#define MOTOR1_DIR_PIN           (2)
#define MOTOR2_DIR_PIN           (A0)
#define MOTOR1_SPEED_PIN         (9)
#define MOTOR2_SPEED_PIN         (10)

#define LOOP_INTERVAL        (10)   // Unit: ms.
#define DURATION_BATTERY_LOW (1000) // Unit: ms. Alert battery low to Core if the state is kept in this duration

#define MOTOR_HZ      (8000)
#define MAX_DUTY_CNT  (F_CPU / MOTOR_HZ) // 1000
#define MAX_SPEED     (8000)             // Keep min/max spec same as version 0x0011
#define DEFAULT_SPEED (0)
#define CHANNELS      (2)

const uint8_t  USER_FW_MAJOR_VER = 0x00;
const uint8_t  USER_FW_MINOR_VER = 0x13;
const uint16_t USER_FW_VER       = (((uint16_t)(USER_FW_MAJOR_VER) << 8) + ((uint16_t)(USER_FW_MINOR_VER)));
const uint32_t BRANCH_TYPE       = 0x00000002; // Branch index number on vivitainc/ViviParts.git

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_ANALOG_2BYTES, -1 * MAX_SPEED, MAX_SPEED,
   DEFAULT_SPEED}, // 1: Motor1 Control
  {DcGroup_t::DC_GROUP_2, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_ANALOG_2BYTES, -1 * MAX_SPEED, MAX_SPEED,
   DEFAULT_SPEED},                                                                              // 2: Motor2 Control
  {DcGroup_t::DC_GROUP_FOR_SYSTEM, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_BOOLEAN, 0, 1}, // 3: Is Battery Low
};

const uint8_t dir_pins[CHANNELS] = {
  MOTOR1_DIR_PIN,
  MOTOR2_DIR_PIN,
};
const uint8_t speed_pins[CHANNELS] = {
  MOTOR1_SPEED_PIN,
  MOTOR2_SPEED_PIN,
};

static inline void analogWrite_(const uint8_t pin, const uint16_t value) {
  const uint16_t mappedValue = map(value, 0, MAX_SPEED, 0, MAX_DUTY_CNT);

  if (mappedValue == 0) {
    digitalWrite(pin, LOW);
  } else if (mappedValue == MAX_DUTY_CNT) {
    digitalWrite(pin, HIGH);
  } else {
    if (pin == MOTOR1_SPEED_PIN) {
      bitSet(TCCR1A, COM1A1);
      OCR1A = mappedValue;
    } else if (pin == MOTOR2_SPEED_PIN) {
      bitSet(TCCR1A, COM1B1);
      OCR1B = mappedValue;
    } else {
      // do nothing
    }
  }
}

static inline void setDirection(const uint8_t pin, const int16_t value) {
  digitalWrite(pin, (value > 0) ? HIGH : LOW);
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

  // Activate RESET# at once for alert circuit to work
  digitalWrite(BATTERY_ALERT_RESET_PIN, LOW);
  digitalWrite(BATTERY_ALERT_RESET_PIN, HIGH);

  for (int i = 0; i < CHANNELS; i++) {
    digitalWrite(dir_pins[i], LOW);
    pinMode(dir_pins[i], OUTPUT);
    pinMode(speed_pins[i], OUTPUT);
  }

  TCCR1A = bit(COM1A1) | bit(COM1B1) | // no inverting
           bit(WGM11);
  TCCR1B = bit(WGM13) | bit(WGM12) | // fast PWM, TOP=ICR1
           bit(CS10);                // no prescaling
  ICR1 = MAX_DUTY_CNT;               // TOP counter value

  for (uint8_t ch = 0; ch < CHANNELS; ch++) {
    setDirection(dir_pins[ch], DEFAULT_SPEED);
    analogWrite_(speed_pins[ch], DEFAULT_SPEED);
  }
}

void loop() {
  static int16_t prev_speeds[CHANNELS] = {
    DEFAULT_SPEED,
    DEFAULT_SPEED,
  };
  const AvailableNum_t cnt = Vivicore.available();

  delay(LOOP_INTERVAL);

  detectLowBattery();

  for (uint8_t i = 0; i < cnt.scaler; i++) {
    const ScalerData_t scaler = Vivicore.read();
    const int16_t      speed  = static_cast<int16_t>(scaler.data);

    if (scaler.success && (0 < scaler.dc_n) && (scaler.dc_n <= CHANNELS)) {
      const uint8_t ch = scaler.dc_n - 1;
      if (speed != prev_speeds[ch]) {
        prev_speeds[ch] = speed;
        setDirection(dir_pins[ch], speed);
        analogWrite_(speed_pins[ch], (uint16_t)abs(speed));
      }
    }

    DebugPlainPrint0("success:");
    DebugPlainPrint0(scaler.success);
    DebugPlainPrint0(", speed");
    DebugPlainPrint0(scaler.dc_n);
    DebugPlainPrint0(":");
    DebugPlainPrintln0(speed);
  }
}
