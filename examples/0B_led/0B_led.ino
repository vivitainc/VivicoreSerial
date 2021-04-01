#define MIN_LIBRARY_VER_BUILD_NO (0x0004)
#include <VivicoreSerial.h>

#define LED_HZ          (1000)
#define LED_VCC_HZ      (32 * 1000)
#define MAX_LIGHT_8BIT  (0xFF)
#define MAX_LIGHT_16BIT (F_CPU / LED_HZ)
#define DEFAULT_LIGHT   (0)
#if defined(BOARD_REV) && (BOARD_REV >= BOARD_REV_FP1_DVT)
#define MAX_LIGHT_R     (6234)            // For 5500K white
#define MAX_LIGHT_G     (6909)            // For 5500K white
#define MAX_LIGHT_B     (MAX_LIGHT_8BIT)  // For 5500K white
#else
#define MAX_LIGHT_R     (MAX_LIGHT_8BIT)
#define MAX_LIGHT_G     (MAX_LIGHT_16BIT)
#define MAX_LIGHT_B     (MAX_LIGHT_16BIT)
#endif

#define LED_VCC_PIN     (2)
#if defined(BOARD_REV) && (BOARD_REV >= BOARD_REV_FP1_DVT)
#define LED_R_PIN       (9)
#define LED_G_PIN       (10)
#define LED_B_PIN       (3)
#else
#define LED_R_PIN       (3)
#define LED_G_PIN       (10)
#define LED_B_PIN       (9)
#endif

const uint16_t USER_FW_VER = 0x000F;
const uint32_t BRANCH_TYPE = 0x0000000B;  // Branch index number on vivitainc/ViviParts.git

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DC_GROUP_1, DC_NATURE_IN, DC_TYPE_ANALOG_2BYTES, 0, MAX_LIGHT_R, DEFAULT_LIGHT}, // 1: Red
  {DC_GROUP_1, DC_NATURE_IN, DC_TYPE_ANALOG_2BYTES, 0, MAX_LIGHT_G, DEFAULT_LIGHT}, // 2: Green
  {DC_GROUP_1, DC_NATURE_IN, DC_TYPE_ANALOG_2BYTES, 0, MAX_LIGHT_B, DEFAULT_LIGHT}, // 3: Blue
};
uint8_t statbuffer[] = {
  1, 0, 0, // 1: Red
  2, 0, 0, // 2: Green
  3, 0, 0, // 3: Blue (Needs 2bytes buffer exceeding 127 as signed max value of 1byte analog)
};

uint16_t led_r = DEFAULT_LIGHT;
uint16_t led_g = DEFAULT_LIGHT;
uint16_t led_b = DEFAULT_LIGHT;

static inline void analogWrite_(const int pin, const uint16_t value) {
  const uint16_t max_value =
#if defined(BOARD_REV) && (BOARD_REV >= BOARD_REV_FP1_DVT)
    (pin == LED_B_PIN) ? MAX_LIGHT_8BIT : MAX_LIGHT_16BIT;
#else
    (pin == LED_R_PIN) ? MAX_LIGHT_8BIT : MAX_LIGHT_16BIT;
#endif
  const bool is_max = (value == max_value);

  if (value == 0) {
    digitalWrite(pin, LOW);
  } else if (is_max) {
    digitalWrite(pin, HIGH);
  } else {
#if defined(BOARD_REV) && (BOARD_REV >= BOARD_REV_FP1_DVT)
    if (pin == LED_R_PIN) {
#else
    if (pin == LED_B_PIN) {
#endif
      bitSet(TCCR1A, COM1A1);
      OCR1A = value;
    } else if (pin == LED_G_PIN) {
      bitSet(TCCR1A, COM1B1);
      OCR1B = value;
#if defined(BOARD_REV) && (BOARD_REV >= BOARD_REV_FP1_DVT)
    } else if (pin == LED_B_PIN) {
#else
    } else if (pin == LED_R_PIN) {
#endif
      bitSet(TCCR2A, COM2B1);
      OCR2B = (uint8_t)value;
    } else {
      // do nothing
    }
  }
}

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);

  pinMode(LED_VCC_PIN, OUTPUT);
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  TCCR3A = bit(COM3B1) | bit(WGM31);               // no inverting, OC3B only connected
  TCCR3B = bit(WGM33) | bit(WGM32) | bit(CS30);    // fast PWM, TOP=ICR3, no prescaling
  ICR3 = F_CPU / LED_VCC_HZ;                       // TOP counter value
  OCR3B = F_CPU / LED_VCC_HZ / 2;                  // OC3B (D2/PD2) only output with duty cycle 50%
  bitSet(PORTD, 2);                                // When not using the Output Compare Modulator, PORTD2 must also be set in order to enable the output.

  TCCR1A = bit(COM1A1) | bit(COM1B1) | bit(WGM11); // no inverting, OC1A and OC1B connected
  TCCR1B = bit(WGM13) | bit(WGM12) | bit(CS10);    // fast PWM, TOP=ICR1, no prescaling
  ICR1 = MAX_LIGHT_16BIT;                          // TOP counter value for 1kHz

  TCCR2A = bit(COM2B1) | bit(WGM21) | bit(WGM20);  // no inverting, OC2B only connected, fast PWM, TOP=0xFF
  TCCR2B = bit(CS21);                              // clk/8 prescaling for about 4kHz higher frequency than another 16bit timers

  analogWrite_(LED_R_PIN, led_r);
  analogWrite_(LED_G_PIN, led_g);
  analogWrite_(LED_B_PIN, led_b);
}

void loop() {
  uint8_t recv_cnt = 0;

  delay(10);

  while (Vivicore.available()) {
    uint8_t byte = Vivicore.read();
    if (recv_cnt < sizeof(statbuffer)) {
      statbuffer[recv_cnt++] = byte;
    }
  }

  if (recv_cnt > 0) {
    int i = 0;

    while (i < (recv_cnt - 1)) {
      const uint8_t dc_number = statbuffer[i++];

      switch (dc_number) {
      case 1:
        led_r = statbuffer[i++];
        led_r <<= 8;
        led_r |= statbuffer[i++];
        break;
      case 2:
        led_g = statbuffer[i++];
        led_g <<= 8;
        led_g |= statbuffer[i++];
        break;
      case 3:
        led_b = statbuffer[i++];
        led_b <<= 8;
        led_b |= statbuffer[i++];
        break;
      default:
        // Exit while loop
        i += recv_cnt;
        break;
      }
    }

    analogWrite_(LED_R_PIN, led_r);
    analogWrite_(LED_G_PIN, led_g);
    analogWrite_(LED_B_PIN, led_b);
  }
}
