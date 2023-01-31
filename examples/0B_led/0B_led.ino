#define MIN_LIBRARY_VER_BUILD_NO (0x0012)
#include <VivicoreSerial.h>

#define LED_HZ          (1000)
#define LED_VCC_HZ      (32 * 1000)
#define MAX_LIGHT_8BIT  (0xFF)
#define MAX_LIGHT_16BIT (F_CPU / LED_HZ)
#define DEFAULT_LIGHT   (0)
#define DEFAULT_EN      (false)
#if defined(BOARD_REV) && (BOARD_REV >= BOARD_REV_FP1_DVT)
#  define MAX_LIGHT_R (6234)           // For 5500K white
#  define MAX_LIGHT_G (6909)           // For 5500K white
#  define MAX_LIGHT_B (MAX_LIGHT_8BIT) // For 5500K white
#else
#  define MAX_LIGHT_R (MAX_LIGHT_8BIT)
#  define MAX_LIGHT_G (MAX_LIGHT_16BIT)
#  define MAX_LIGHT_B (MAX_LIGHT_16BIT)
#endif

#define LED_VCC_PIN (2)
#if defined(BOARD_REV) && (BOARD_REV >= BOARD_REV_FP1_DVT)
#  define LED_R_PIN (9)
#  define LED_G_PIN (10)
#  define LED_B_PIN (3)
#else
#  define LED_R_PIN (3)
#  define LED_G_PIN (10)
#  define LED_B_PIN (9)
#endif
#define N_RGB     (3)
#define EXP_GAMMA ((float)2.2)

enum dcInfoNumber_t {
  nEnabled = 1,
  nRed,
  nGreen,
  nBlue,
};

const uint8_t  USER_FW_MAJOR_VER = 0x01;
const uint8_t  USER_FW_MINOR_VER = 0x02;
const uint16_t USER_FW_VER       = (((uint16_t)(USER_FW_MAJOR_VER) << 8) + ((uint16_t)(USER_FW_MINOR_VER)));
const uint32_t BRANCH_TYPE       = 0x0000000B; // Branch index number on vivitainc/ViviParts.git

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_BOOLEAN, false, true, DEFAULT_EN}, // 1: Enabled
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_ANALOG_2BYTES, 0, MAX_LIGHT_R,
   DEFAULT_LIGHT}, // 2: Red
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_ANALOG_2BYTES, 0, MAX_LIGHT_G,
   DEFAULT_LIGHT}, // 3: Green
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_ANALOG_2BYTES, 0, MAX_LIGHT_B,
   DEFAULT_LIGHT}, // 4: Blue
};

static const int pins[N_RGB] = {
  LED_R_PIN,
  LED_G_PIN,
  LED_B_PIN,
};

static uint16_t lights[N_RGB] = {
  DEFAULT_LIGHT,
  DEFAULT_LIGHT,
  DEFAULT_LIGHT,
};

static bool enabled = DEFAULT_EN;

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

static inline void correctGamma(const uint16_t *rgb_in, uint16_t *rgb_out) {
  static const uint16_t max_lights[N_RGB] = {
    MAX_LIGHT_R,
    MAX_LIGHT_G,
    MAX_LIGHT_B,
  };

  for (uint8_t i = 0; i < N_RGB; i++) {
    rgb_out[i] = (uint16_t)roundf((float)max_lights[i] * pow((float)rgb_in[i] / (float)max_lights[i], EXP_GAMMA));
  }
}

static inline void analogWriteGamma(const uint16_t *rgb_in, const bool enabled_) {
  uint16_t rgb_out[N_RGB] = {};
  if (enabled_) {
    correctGamma(rgb_in, rgb_out);
  }
  for (uint8_t i = 0; i < N_RGB; i++) {
    analogWrite_(pins[i], rgb_out[i]);
  }
}

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);

  pinMode(LED_VCC_PIN, OUTPUT);
  for (uint8_t i = 0; i < N_RGB; i++) {
    pinMode(pins[i], OUTPUT);
  }

  TCCR3A = bit(COM3B1) | bit(WGM31);            // no inverting, OC3B only connected
  TCCR3B = bit(WGM33) | bit(WGM32) | bit(CS30); // fast PWM, TOP=ICR3, no prescaling
  ICR3   = F_CPU / LED_VCC_HZ;                  // TOP counter value
  OCR3B  = F_CPU / LED_VCC_HZ / 2;              // OC3B (D2/PD2) only output with duty cycle 50%
  bitSet(PORTD,
         2); // When not using the Output Compare Modulator, PORTD2 must also be set in order to enable the output.

  TCCR1A = bit(COM1A1) | bit(COM1B1) | bit(WGM11); // no inverting, OC1A and OC1B connected
  TCCR1B = bit(WGM13) | bit(WGM12) | bit(CS10);    // fast PWM, TOP=ICR1, no prescaling
  ICR1   = MAX_LIGHT_16BIT;                        // TOP counter value for 1kHz

  TCCR2A = bit(COM2B1) | bit(WGM21) | bit(WGM20); // no inverting, OC2B only connected, fast PWM, TOP=0xFF
  TCCR2B = bit(CS21); // clk/8 prescaling for about 4kHz higher frequency than another 16bit timers

  analogWriteGamma(lights, enabled);
}

void loop() {
  const AvailableNum_t cnt = Vivicore.available();

  delay(10);

  for (uint8_t i = 0; i < cnt.scaler; i++) {
    const ScalerData_t scaler = Vivicore.read();
    if (scaler.success) {
      switch (scaler.dc_n) {
      case nEnabled:
        enabled = static_cast<bool>(scaler.data);
        break;
      case nRed:
        lights[0] = static_cast<uint16_t>(scaler.data);
        break;
      case nGreen:
        lights[1] = static_cast<uint16_t>(scaler.data);
        break;
      case nBlue:
        lights[2] = static_cast<uint16_t>(scaler.data);
        break;
      default:
        break;
      }

      DebugPlainPrint0("dc_n:");
      DebugPlainPrint0(scaler.dc_n);
      DebugPlainPrint0(", success:");
      DebugPlainPrint0(scaler.success);
      DebugPlainPrint0(", data:");
      DebugPlainPrintln0(scaler.data);
    }
  }

  if (cnt.scaler > 0) {
    analogWriteGamma(lights, enabled);
  }
}
