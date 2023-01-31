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

#define MOTOR_HZ        (8000)
#define MAX_DUTY_CNT    (F_CPU / MOTOR_HZ) // 1000
#define MAX_SPEED       (8000)             // Keep min/max spec same as version 0x0011
#define DEFAULT_SPEED   (0)
#define DEFAULT_EN      (false)
#define DEFAULT_REVERSE (false) // Counter-clockwise
#define CHANNELS        (2)
#define INVALID_CHANNEL (CHANNELS)

enum dcInfoNumber_t {
  nMotor1Enabled = 1,
  nMotor1Reverse,
  nMotor1Speed,
  nMotor2Enabled,
  nMotor2Reverse,
  nMotor2Speed,
  nBatteryLow,
};

const uint8_t  USER_FW_MAJOR_VER = 0x01;
const uint8_t  USER_FW_MINOR_VER = 0x01;
const uint16_t USER_FW_VER       = (((uint16_t)(USER_FW_MAJOR_VER) << 8) + ((uint16_t)(USER_FW_MINOR_VER)));
const uint32_t BRANCH_TYPE       = 0x00000002; // Branch index number on vivitainc/ViviParts.git

const dcInfo_t dc_info[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_BOOLEAN, false, true,
   DEFAULT_EN}, // 1: Motor1 ON/OFF
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_BOOLEAN, false, true,
   DEFAULT_REVERSE}, // 2: Motor1 Reverse
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_ANALOG_2BYTES, -1 * MAX_SPEED, MAX_SPEED,
   DEFAULT_SPEED}, // 3: Motor1 Speed
  {DcGroup_t::DC_GROUP_2, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_BOOLEAN, false, true,
   DEFAULT_EN}, // 4: Motor2 ON/OFF
  {DcGroup_t::DC_GROUP_2, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_BOOLEAN, false, true,
   DEFAULT_REVERSE}, // 5: Motor2 Reverse
  {DcGroup_t::DC_GROUP_2, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_ANALOG_2BYTES, -1 * MAX_SPEED, MAX_SPEED,
   DEFAULT_SPEED}, // 6: Motor2 Speed
  {DcGroup_t::DC_GROUP_FOR_SYSTEM, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_BOOLEAN, false,
   true}, // 7: Is Battery Low
};

const uint8_t dir_pins[CHANNELS] = {
  MOTOR1_DIR_PIN,
  MOTOR2_DIR_PIN,
};
const uint8_t speed_pins[CHANNELS] = {
  MOTOR1_SPEED_PIN,
  MOTOR2_SPEED_PIN,
};

static inline uint8_t mapDcNumberToChannel(const uint8_t dc_n) {
  if (nMotor1Enabled <= dc_n && dc_n <= nMotor1Speed) {
    return 0;
  } else if (nMotor2Enabled <= dc_n && dc_n <= nMotor2Speed) {
    return 1;
  }

  return INVALID_CHANNEL;
}

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
      Vivicore.write(nBatteryLow, isBatteryLow);
    }

    Vivicore.flush();

    DebugPlainPrint0("duration:");
    DebugPlainPrint0(batteryLowDuration);
    DebugPlainPrint0(", low:");
    DebugPlainPrintln0(isBatteryLow);
  }
}

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dc_info, countof(dc_info), MIN_LIBRARY_VER_BUILD_NO);

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
    analogWrite_(speed_pins[ch], (uint16_t)abs(DEFAULT_SPEED));
  }
}

void loop() {
  static int16_t cur_velocity[CHANNELS] = {
    DEFAULT_SPEED,
    DEFAULT_SPEED,
  };
  static int16_t cur_speeds[CHANNELS] = {
    DEFAULT_SPEED,
    DEFAULT_SPEED,
  };
  static bool cur_enabled[CHANNELS] = {
    DEFAULT_EN,
    DEFAULT_EN,
  };
  static bool cur_reverse[CHANNELS] = {
    DEFAULT_REVERSE,
    DEFAULT_REVERSE,
  };
  const AvailableNum_t cnt = Vivicore.available();

  delay(LOOP_INTERVAL);

  detectLowBattery();

  for (uint8_t i = 0; i < cnt.scaler; i++) {
    const ScalerData_t scaler = Vivicore.read();

    DebugPlainPrint0("success:");
    DebugPlainPrint0(scaler.success);

    if (scaler.success) {
      const uint8_t ch = mapDcNumberToChannel(scaler.dc_n);

      if (ch < CHANNELS) {
        switch (scaler.dc_n) {
        case nMotor1Enabled:
        case nMotor2Enabled:
          cur_enabled[ch] = static_cast<bool>(scaler.data);
          break;
        case nMotor1Reverse:
        case nMotor2Reverse:
          cur_reverse[ch] = static_cast<bool>(scaler.data);
          break;
        case nMotor1Speed:
        case nMotor2Speed:
          cur_speeds[ch] = static_cast<int16_t>(scaler.data);
          break;
        default:
          break;
        }

        const int16_t velocity = (cur_enabled[ch] ? cur_speeds[ch] : 0) * (cur_reverse[ch] ? -1 : 1);
        if (velocity != cur_velocity[ch]) {
          cur_velocity[ch] = velocity;
          setDirection(dir_pins[ch], velocity);
          analogWrite_(speed_pins[ch], (uint16_t)abs(velocity));
        }

        DebugPlainPrint0(", speed");
        DebugPlainPrint0(scaler.dc_n);
        DebugPlainPrint0(":");
        DebugPlainPrint0(velocity);
      } else {
        DebugPlainPrint0(", invalid channel:");
        DebugPlainPrint0(ch);
      }
    }

    DebugPlainPrintln0("");
  }
}
