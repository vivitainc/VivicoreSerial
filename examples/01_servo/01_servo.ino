#define MIN_LIBRARY_VER_BUILD_NO (0x0004)
#include <VivicoreSerial.h>

#define ANGLE_RANGE    (180)
#define CYCLE_HZ       (50)
#define PULSE_MIN_US   (900)                          // the shortest pulse sent to a servo (544 is defined as MIN_PULSE_WIDTH on Servo.h)
#define PULSE_MAX_US   (2100)                         // the longest pulse sent to a servo (2400 is defined as MAX_PULSE_WIDTH on Servo.h)
#define MAX_CNT        (F_CPU / 8 / CYCLE_HZ)         // fclk 8MHz, prescaler 8, freq of servo 50Hz
#define US_PER_CNT     (1000000 / CYCLE_HZ / MAX_CNT)
#define CNT_PER_ANGLE  ((PULSE_MAX_US - PULSE_MIN_US) / US_PER_CNT / ANGLE_RANGE)
#define ANGLE_MIN_CNT  (PULSE_MIN_US / US_PER_CNT)    // equivalent 0 degree
#define ANGLE_MAX_CNT  (PULSE_MAX_US / US_PER_CNT)    // equivalent 180 degrees
#define DEFAULT_CNT    ((ANGLE_MAX_CNT + ANGLE_MIN_CNT) / 2)

#define SERVO1_ANGLE_PIN         (10)
#define SERVO2_ANGLE_PIN         (9)
#define BATTERY_ALERT_RESET_PIN  (A1)  // LOW active
#define BATTERY_ALERT_DETECT_PIN (A2)  // LOW active

#define LOOP_INTERVAL        (10)    // Unit: ms.
#define DURATION_BATTERY_LOW (1000)  // Unit: ms. Alert battery low to Core if the state is kept in this duration

const uint16_t USER_FW_VER = 0x0008;
const uint32_t BRANCH_TYPE = 0x00000001;  // Branch index number on vivitainc/ViviParts.git

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max, data_ini}
  {DC_GROUP_1,          DC_NATURE_IN,  DC_TYPE_ANALOG_2BYTES, ANGLE_MIN_CNT, ANGLE_MAX_CNT, DEFAULT_CNT}, // 1: S1 Angle
  {DC_GROUP_2,          DC_NATURE_IN,  DC_TYPE_ANALOG_2BYTES, ANGLE_MIN_CNT, ANGLE_MAX_CNT, DEFAULT_CNT}, // 2: S2 Angle
  {DC_GROUP_FOR_SYSTEM, DC_NATURE_OUT, DC_TYPE_BOOLEAN,       0,             1},                          // 3: Is Battery Low
};
uint8_t statbuffer[] = {
  1, 0, 0,           // 1: S1 Angle
  2, 0, 0,           // 2: S2 Angle
};
uint8_t battery_status[] = {
  3, (uint8_t)false, // 3: Is Battery Low
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
  pinMode(SERVO1_ANGLE_PIN, OUTPUT);
  pinMode(SERVO2_ANGLE_PIN, OUTPUT);

  // Activate RESET# at once for alert circuit to work
  digitalWrite(BATTERY_ALERT_RESET_PIN, LOW);
  digitalWrite(BATTERY_ALERT_RESET_PIN, HIGH);

  TCCR1A = bit(COM1A1) | bit(COM1B1) | // no inverting
           bit(WGM11);
  TCCR1B = bit(WGM13) | bit(WGM12) |   // fast PWM, TOP=ICR1
           bit(CS11);                  // clk/8 prescaler
  ICR1 = MAX_CNT;                      // TOP counter value
}

void loop() {
  static uint16_t angleCnt[] = {DEFAULT_CNT, DEFAULT_CNT};
  static uint16_t angleCntPrev[] = {0, 0};
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
        angleCnt[0] = statbuffer[i++];
        angleCnt[0] <<= 8;
        angleCnt[0] |= statbuffer[i++];
        break;
      case 2:
        angleCnt[1] = statbuffer[i++];
        angleCnt[1] <<= 8;
        angleCnt[1] |= statbuffer[i++];
        break;
      default:
        // Exit while loop
        i += recvCnt;
        break;
      }
    } // while
  }

  if (angleCntPrev[0] != angleCnt[0]) {
    analogWriteWithProtection(SERVO1_ANGLE_PIN, angleCnt[0]);
    angleCntPrev[0] = angleCnt[0];
    DebugPlainPrint0("angle0:");
    DebugPlainPrintln0(angleCnt[0]);
  }

  if (angleCntPrev[1] != angleCnt[1]) {
    analogWriteWithProtection(SERVO2_ANGLE_PIN, angleCnt[1]);
    angleCntPrev[1] = angleCnt[1];
    DebugPlainPrint0("angle1:");
    DebugPlainPrintln0(angleCnt[1]);
  }
}
