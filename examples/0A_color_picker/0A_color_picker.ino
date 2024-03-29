#define MIN_LIBRARY_VER_BUILD_NO (0x0012)
#include <VivicoreSerial.h>
#include <Adafruit_APDS9960.h>
#include <math.h>

#define MAX_RGB       (255)
#define MAX_HUE       (359)
#define MAX_SAT       (255)
#define MAX_VAL       (255)
#define LIGHT_FACTOR  (10)
#define MAX_LIGHT     (255 * LIGHT_FACTOR)
#define DEFAULT_LIGHT (MAX_LIGHT * 3 / 10)
#define VCC_HZ        (32 * 1000)
#define COLORS        (8 + 2) // Color number + origin + end point

#define PROXIMITY_THRESHOLD (255)

#define VCC_PIN (9)
#define LED_PIN (3)

enum {
  RGB_R = 0,
  RGB_G,
  RGB_B,
  RGB_NUM,
};
enum {
  HSV_H = 0,
  HSV_S,
  HSV_V,
  HSV_NUM,
};
enum dcInfoNumber_t {
  nRed = 1,
  nGreen,
  nBlue,
  nProximity,
  nLed,
};

const uint8_t  USER_FW_MAJOR_VER = 0x01;
const uint8_t  USER_FW_MINOR_VER = 0x01;
const uint16_t USER_FW_VER       = (((uint16_t)(USER_FW_MAJOR_VER) << 8) + ((uint16_t)(USER_FW_MINOR_VER)));
const uint32_t BRANCH_TYPE       = 0x0000000A; // Branch index number on vivitainc/ViviParts.git

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max, data_ini}
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_2BYTES, 0, MAX_RGB}, // 1: Red
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_2BYTES, 0, MAX_RGB}, // 2: Green
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_2BYTES, 0, MAX_RGB}, // 3: Blue
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_BOOLEAN, false, true},      // 4: Proximity
  {DcGroup_t::DC_GROUP_2, DcNature_t::DC_NATURE_IN, DcType_t::DC_TYPE_ANALOG_2BYTES, 0, MAX_LIGHT,
   DEFAULT_LIGHT}, // 5: Light LED
};

// Color conversion table written on this esa.
// https://vivita.esa.io/posts/6726#%E8%89%B2%E7%A7%BB%E5%8B%95
static const long h_in[COLORS] = {
  0, 31, 51, 95, 198, 224, 280, 320, 356, 359,
};
static const long h_out[COLORS] = {
  0, 29, 50, 122, 191, 237, 283, 323, 357, 359,
};

Adafruit_APDS9960 apds9960;

static inline float mapf(const float x, const float in_min, const float in_max, const float out_min,
                         const float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline void analogWriteToLed(const uint16_t value) {
  if (value == 0) {
    digitalWrite(LED_PIN, LOW);
  } else if (value == MAX_LIGHT) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    // uint8_t rawVal = (uint8_t)roundf((float)value / (float)LIGHT_FACTOR); // More accurate
    uint8_t rawVal = (uint8_t)(value / LIGHT_FACTOR);
    bitSet(TCCR2A, COM2B1);
    OCR2B = rawVal;
  }
}

static inline uint16_t convertHue(const uint16_t hue) {
  uint8_t index = 0;

  for (index = 0; index < COLORS - 1; index++) {
    if (h_in[index] <= hue && hue <= h_in[index + 1]) {
      break;
    }
  }

  return (uint16_t)map(hue, h_in[index], h_in[index + 1], h_out[index], h_out[index + 1]);
}

static inline float convertHue(const float hue) {
  uint8_t index = 0;

  for (index = 0; index < COLORS - 1; index++) {
    if ((float)h_in[index] <= hue && hue <= (float)h_in[index + 1]) {
      break;
    }
  }

  return mapf(hue, (float)h_in[index], (float)h_in[index + 1], (float)h_out[index], (float)h_out[index + 1]);
}

static inline uint16_t convertRoundedHue(const uint16_t hue) {
  return roundf(convertHue((float)hue));
}

static inline void convertHsv(const uint16_t *rgb_raw, uint16_t *hsv) {
  static const float darkc[RGB_NUM] = {
    96.0,
    360.0,
    284.0,
  };
  static const float to_non_interfer[RGB_NUM][RGB_NUM] = {
    {1.60, -0.40, -0.03},
    {-0.10, 1.55, -0.45},
    {-0.24, -0.60, 1.50},
  };
  static const float gc_div[RGB_NUM] = {
    5000.0,
    6000.0,
    5000.0,
  };
  static const float gc_offset[RGB_NUM] = {
    22.0,
    29.0,
    26.0,
  };
  int16_t hue                = 0;
  float   rgb_darkc[RGB_NUM] = {};
  float   rgb[RGB_NUM]       = {};
  int16_t rgb_gc[RGB_NUM]    = {};

  for (uint8_t i = 0; i < RGB_NUM; i++) {
    rgb_darkc[i] = (float)rgb_raw[i] - darkc[i];
    if (rgb_darkc[i] < 0.0) {
      rgb_darkc[i] = 0.0;
    }
  }

  for (uint8_t i = 0; i < RGB_NUM; i++) {
    const float *row = to_non_interfer[i];
    for (uint8_t j = 0; j < RGB_NUM; j++) {
      rgb[i] += row[j] * rgb_darkc[j];
    }
    rgb[i]    = constrain(rgb[i], 0.0, 10000.0);
    rgb_gc[i] = (int16_t)constrain(255.0 * pow(rgb[i] / gc_div[i], 0.35) - gc_offset[i], 0.0, 255.0);
  }

  const int16_t max_ = max(rgb_gc[RGB_R], max(rgb_gc[RGB_G], rgb_gc[RGB_B]));
  const int16_t min_ = min(rgb_gc[RGB_R], min(rgb_gc[RGB_G], rgb_gc[RGB_B]));
  hsv[HSV_S]         = (uint8_t)((max_) ? (255L * (max_ - min_) / max_) : 0);
  hsv[HSV_V]         = (uint8_t)max_;

  if (max_ == min_) {
    hue = 0;
  } else if (max_ == rgb_gc[RGB_R]) {
    hue = 60L * (rgb_gc[RGB_G] - rgb_gc[RGB_B]) / (max_ - min_);
  } else if (max_ == rgb_gc[RGB_G]) {
    hue = 60L * (rgb_gc[RGB_B] - rgb_gc[RGB_R]) / (max_ - min_) + 120;
  } else {
    hue = 60L * (rgb_gc[RGB_R] - rgb_gc[RGB_G]) / (max_ - min_) + 240;
  }
  if (hue < 0) {
    hue += 360;
  }

  hsv[HSV_H] = (uint16_t)hue;
}

static inline void convertRgb(const uint16_t *hsv, uint8_t *rgb) {
  const float   h      = static_cast<float>(hsv[0]);
  const float   s      = constrain(static_cast<float>(hsv[1]) / static_cast<float>(MAX_SAT), 0.0, 1.0);
  const float   v      = constrain(static_cast<float>(hsv[2]) / static_cast<float>(MAX_VAL), 0.0, 1.0);
  const uint8_t v_byte = static_cast<uint8_t>(hsv[2]);

  if (abs(s) <= (1.0 / static_cast<float>(1 << 12))) { // shade of gray
    memset(rgb, v_byte, 3);
    return;
  }

  const float hx = (hsv[0] >= 360) ? 0.0 : h / 60.0;
  const float w  = floor(hx);
  const float f  = hx - w;

  const uint8_t p = static_cast<uint8_t>(round((1.0 - s) * v * 255.0));
  const uint8_t q = static_cast<uint8_t>(round((1.0 - (s * f)) * v * 255.0));
  const uint8_t t = static_cast<uint8_t>(round((1.0 - (s * (1.0 - f))) * v * 255.0));
  uint8_t       r, g, b;

  switch (static_cast<uint8_t>(w)) {
  case 0:
    r = v_byte;
    g = t;
    b = p;
    break;
  case 1:
    r = q;
    g = v_byte;
    b = p;
    break;
  case 2:
    r = p;
    g = v_byte;
    b = t;
    break;
  case 3:
    r = p;
    g = q;
    b = v_byte;
    break;
  case 4:
    r = t;
    g = p;
    b = v_byte;
    break;
  default:
    r = v_byte;
    g = p;
    b = q;
    break;
  }

  rgb[RGB_R] = r;
  rgb[RGB_G] = g;
  rgb[RGB_B] = b;
}

static inline void controlLed(void) {
  const AvailableNum_t cnt = Vivicore.available();

  for (uint8_t i = 0; i < cnt.scaler; i++) {
    const ScalerData_t scaler = Vivicore.read();
    if (scaler.success && (scaler.dc_n == nLed)) {
      analogWriteToLed(static_cast<uint16_t>(scaler.data));
    }
    DebugPlainPrint0("dc_n:");
    DebugPlainPrint0(scaler.dc_n);
    DebugPlainPrint0(", success:");
    DebugPlainPrint0(scaler.success);
    DebugPlainPrint0(", data:");
    DebugPlainPrintln0(scaler.data);
  }
}

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);
  if (apds9960.begin()) {
    apds9960.enableColor(true);
    apds9960.setADCGain(APDS9960_AGAIN_64X);
    // arg type is uint16_t and calclate ATIME reg value like below internally
    // ATIME reg: 256 - (iTimeMS / 2.78)
    apds9960.setADCIntegrationTime(28);
    apds9960.enableProximity(true);
  } else {
    DebugPlainPrintln0("Error: initialization");
  }

  pinMode(VCC_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  TCCR1A = bit(COM1A1) | bit(WGM11);            // no inverting, OC1A only connected
  TCCR1B = bit(WGM13) | bit(WGM12) | bit(CS10); // fast PWM, TOP=ICR1, no prescaling
  ICR1   = F_CPU / VCC_HZ;                      // TOP counter value for 32kHz
  OCR1A  = F_CPU / VCC_HZ / 2;                  // OC1A (D9/PB1) only output with duty cycle 50%

  TCCR2A = bit(COM2B1) | bit(WGM21) | bit(WGM20); // no inverting, OC2B only connected, fast PWM, TOP=0xFF
  TCCR2B = bit(CS21);                             // clk/8 prescaling for about 4kHz
  analogWriteToLed(DEFAULT_LIGHT);                // OC2B (D3/PD3) only output
}

void loop() {
  static bool     on_target_prev    = false;
  static uint16_t rgb_prev[HSV_NUM] = {};
  uint16_t        hue               = 0;
  uint16_t        hsv[HSV_NUM]      = {};
  uint16_t        rgb_raw[RGB_NUM]  = {};
  uint8_t         rgb[RGB_NUM]      = {};
  uint16_t        c                 = 0;

  delay(30);
  controlLed();

  const uint8_t proximity = apds9960.readProximity();
  const bool    on_target = (proximity >= PROXIMITY_THRESHOLD);
  if (on_target != on_target_prev) {
    on_target_prev = on_target;
    Vivicore.write(nProximity, on_target);
  }

  while (!apds9960.colorDataReady()) {
    delay(5);
  }
  apds9960.getColorData(&rgb_raw[RGB_R], &rgb_raw[RGB_G], &rgb_raw[RGB_B], &c);

  convertHsv(rgb_raw, hsv);
  hue        = hsv[HSV_H];
  hsv[HSV_H] = convertRoundedHue(hue);
  convertRgb(hsv, rgb);

  for (uint8_t i = 0; i < RGB_NUM; i++) {
    if (rgb_prev[i] != rgb[i]) {
      rgb_prev[i] = rgb[i];
      switch (i) {
      case RGB_R:
        Vivicore.write(nRed, rgb[i]);
        break;
      case RGB_G:
        Vivicore.write(nGreen, rgb[i]);
        break;
      case RGB_B:
        Vivicore.write(nBlue, rgb[i]);
        break;
      default:
        break;
      }
    }
  }

  Vivicore.flush();

  DebugPlainPrint0("raw:");
  DebugHexPrint0(rgb[RGB_R]);
  DebugPlainPrint0(",");
  DebugHexPrint0(rgb[RGB_G]);
  DebugPlainPrint0(",");
  DebugHexPrint0(rgb[RGB_B]);
  DebugPlainPrint0(",");
  DebugHexPrint0(c >> 8);
  DebugHexPrint0(c);
  DebugPlainPrintln0("");
  DebugPlainPrint0("hsv:");
  DebugHexPrint0(hsv[HSV_H] >> 8);
  DebugHexPrint0(hsv[HSV_H]);
  DebugPlainPrint0(",");
  DebugHexPrint0(hsv[HSV_S] >> 8);
  DebugHexPrint0(hsv[HSV_S]);
  DebugPlainPrint0(",");
  DebugHexPrint0(hsv[HSV_V] >> 8);
  DebugHexPrint0(hsv[HSV_V]);
  DebugPlainPrintln0("");
  DebugPlainPrint0("hue:");
  DebugHexPrint0(hue >> 8);
  DebugHexPrint0(hue);
  DebugPlainPrint0(",");
  DebugHexPrint0(convertHue(hue) >> 8);
  DebugHexPrint0(convertHue(hue));
  DebugPlainPrintln0(",");
  DebugPlainPrint0("proximity:");
  DebugPlainPrint0(proximity);
  DebugPlainPrintln0("");
}
