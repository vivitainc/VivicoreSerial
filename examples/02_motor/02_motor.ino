#define MIN_LIBRARY_VER_BUILD_NO (0x0008)
#include <VivicoreSerial.h>

#define BATTERY_ALERT_RESET_PIN  (A1)  // LOW active
#define BATTERY_ALERT_DETECT_PIN (A2)  // LOW active
#define MOTOR1_DIR_PIN (2)
#define MOTOR2_DIR_PIN (A0)
#define MOTOR1_SPEED_PIN (9)
#define MOTOR2_SPEED_PIN (10)

#define LOOP_INTERVAL        (10)    // Unit: ms.
#define DURATION_BATTERY_LOW (1000)  // Unit: ms. Alert battery low to Core if the state is kept in this duration

#define MOTOR_HZ  (1000)
#define MAX_DUTY_CNT (F_CPU / MOTOR_HZ)
#define MAX_SPEED (MAX_DUTY_CNT * 65 / 100)  // Limit duty 65% for another motors e.g. FA130
#define DEFAULT_SPEED (0)

const uint16_t USER_FW_VER = 0x000C;
const uint32_t BRANCH_TYPE = 0x00000002;  // Branch index number on vivitainc/ViviParts.git

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DC_GROUP_1,          DC_NATURE_IN,  DC_TYPE_ANALOG_2BYTES, -1 * MAX_SPEED, MAX_SPEED, DEFAULT_SPEED}, // 1: Motor1 Speed & Direction
  {DC_GROUP_2,          DC_NATURE_IN,  DC_TYPE_ANALOG_2BYTES, -1 * MAX_SPEED, MAX_SPEED, DEFAULT_SPEED}, // 2: Motor2 Speed & Direction
  {DC_GROUP_FOR_SYSTEM, DC_NATURE_OUT, DC_TYPE_BOOLEAN,       0,              1},                        // 3: Is Battery Low
};

uint8_t statbuffer[] = {
  1, 0, 0,           // 1: Motor1 Speed & Direction
  2, 0, 0,           // 2: Motor2 Speed & Direction
};
uint8_t battery_status[] = {
  3, (uint8_t)false, // 3: Is Battery Low
};

static inline void analogWrite_(const int pin, const uint16_t value) {
  if (value == 0) {
    digitalWrite(pin, LOW);
  } else if (value == MAX_DUTY_CNT) {
    digitalWrite(pin, HIGH);
  } else {
    if (pin == MOTOR1_SPEED_PIN) {
      bitSet(TCCR1A, COM1A1);
      OCR1A = value;
    } else if (pin == MOTOR2_SPEED_PIN) {
      bitSet(TCCR1A, COM1B1);
      OCR1B = value;
    } else {
      // do nothing
    }
  }
}

static inline void detectLowBattery(void) {
  static uint16_t batteryLowDuration = 0;
  const bool isBatteryLow = (digitalRead(BATTERY_ALERT_DETECT_PIN) == LOW);

  if (isBatteryLow) {
    batteryLowDuration += LOOP_INTERVAL;
    DebugPlainPrint0("battery low:");
    DebugPlainPrintln0(batteryLowDuration);
    digitalWrite(BATTERY_ALERT_RESET_PIN, LOW);
    digitalWrite(BATTERY_ALERT_RESET_PIN, HIGH);
  } else {
    batteryLowDuration = 0;
  }

  if (battery_status[1] != (uint8_t)isBatteryLow) {
    bool shouldNotify = false;
    if (isBatteryLow) {
      if (batteryLowDuration > DURATION_BATTERY_LOW) {
        shouldNotify = true;
      }
    } else {
      shouldNotify = true;
    }

    if (shouldNotify) {
      battery_status[1] = (uint8_t)isBatteryLow;
      Vivicore.write(battery_status, sizeof(battery_status));
    }

    Vivicore.flush();
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

  digitalWrite(MOTOR1_DIR_PIN, LOW);
  digitalWrite(MOTOR2_DIR_PIN, LOW);
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  pinMode(MOTOR1_SPEED_PIN, OUTPUT);
  pinMode(MOTOR2_SPEED_PIN, OUTPUT);

  TCCR1A = bit(COM1A1) | bit(COM1B1) | // no inverting
           bit(WGM11);
  TCCR1B = bit(WGM13) | bit(WGM12) |   // fast PWM, TOP=ICR1
           bit(CS10);                  // no prescaling
  ICR1 = MAX_DUTY_CNT;                 // TOP counter value

  analogWrite_(MOTOR1_SPEED_PIN, DEFAULT_SPEED);
  analogWrite_(MOTOR2_SPEED_PIN, DEFAULT_SPEED);
}

void loop() {
  static int16_t speed1 = 0;
  static int16_t speed2 = 0;
  uint8_t recvCnt = 0;

  delay(LOOP_INTERVAL);

  detectLowBattery();

  while (Vivicore.available()) {
    uint8_t byte = Vivicore.read();
    if (recvCnt < sizeof(statbuffer)) {
      statbuffer[recvCnt++] = byte;
    }
  }

  if (recvCnt > 0) {
    int i = 0;

    while (i < (recvCnt - 1)) {
      const uint8_t dc_number = statbuffer[i++];

      switch (dc_number) {
      case 1:
        speed1 = statbuffer[i++];
        speed1 <<= 8;
        speed1 |= statbuffer[i++];
        digitalWrite(MOTOR1_DIR_PIN, (speed1 >= 0) ? LOW : HIGH);
        break;
      case 2:
        speed2 = statbuffer[i++];
        speed2 <<= 8;
        speed2 |= statbuffer[i++];
        digitalWrite(MOTOR2_DIR_PIN, (speed2 >= 0) ? LOW : HIGH);
        break;
      default:
        // Exit while loop
        i += recvCnt;
        break;
      }
    } // While

    analogWrite_(MOTOR1_SPEED_PIN, (uint16_t)abs(speed1));
    analogWrite_(MOTOR2_SPEED_PIN, (uint16_t)abs(speed2));
  }

  Vivicore.flush();
}
