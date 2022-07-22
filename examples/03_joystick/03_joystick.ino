#define MIN_LIBRARY_VER_BUILD_NO (0x0012)
#include <VivicoreSerial.h>

//#define CENTER_CALIBRATION

#define AD_VALUE_MASK (0x3FF)
#define MAX_AD_VALUE  (AD_VALUE_MASK)
#define MID_AD_VALUE  (MAX_AD_VALUE / 2)
#define MIN_VALUE     (-1 * MID_AD_VALUE)
#define MAX_VALUE     (MID_AD_VALUE)

#define MARGIN_TO_UPDATE (20)       // X/Y axis margin to be updated if exceeding this value
#define MARGIN_ORIGIN    (70)       // n=3, worst case, 45
#define MARGIN_OUTER     (232 + 75) // n=3, worst case, 207, Avg delta b/w DVT and T2 = approx. 75
#define ORIGIN_LIMIT_MIN (MID_AD_VALUE - MARGIN_ORIGIN)
#define ORIGIN_LIMIT_MAX (MID_AD_VALUE + MARGIN_ORIGIN)
#define OUTER_LIMIT_MIN  (MARGIN_OUTER)
#define OUTER_LIMIT_MAX  (MAX_AD_VALUE - MARGIN_OUTER)

#define BUTTON_PIN (2)
#define JX_PIN     (A2)
#define JY_PIN     (A0)

const uint8_t  USER_FW_MAJOR_VER = 0x00;
const uint8_t  USER_FW_MINOR_VER = 0x0D;
const uint16_t USER_FW_VER       = (((uint16_t)(USER_FW_MAJOR_VER) << 8) + ((uint16_t)(USER_FW_MINOR_VER)));
const uint32_t BRANCH_TYPE       = 0x00000003;

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_BOOLEAN, 0, 1}, // 1: PUSH BUTTON
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_2BYTES, MIN_VALUE,
   MAX_VALUE}, // 2: Up & Down
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_ANALOG_2BYTES, MIN_VALUE,
   MAX_VALUE}, // 3: Left & Right
};

struct calcValues_t {
  int16_t calibrated;
  int16_t mapped;
};

int16_t centerX = MID_AD_VALUE;
int16_t centerY = MID_AD_VALUE;

static calcValues_t calcValues(const int16_t rawVal, const int16_t lowerLimit, const int16_t upperLimit) {
  calcValues_t result = {};

  // Trimming center & corner data
  result.calibrated = constrain(rawVal, lowerLimit, upperLimit);
  if ((ORIGIN_LIMIT_MIN < result.calibrated) && (result.calibrated < ORIGIN_LIMIT_MAX)) {
    result.calibrated = MID_AD_VALUE;
  }

  // Map AD values to X/Y axis values
  if (result.calibrated == MID_AD_VALUE) {
    result.mapped = 0;
  } else if (result.calibrated < MID_AD_VALUE) {
    result.mapped = map(result.calibrated, lowerLimit, ORIGIN_LIMIT_MIN, MAX_VALUE, 1);
  } else {
    result.mapped = map(result.calibrated, ORIGIN_LIMIT_MAX, upperLimit, -1, MIN_VALUE);
  }
  return result;
}

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);
  pinMode(BUTTON_PIN, INPUT);

#ifdef CENTER_CALIBRATION
  centerX = (int16_t)analogRead(JX_PIN);
  centerY = (int16_t)analogRead(JY_PIN);
#endif
  DebugPlainPrint0("lower/upperLimit:");
  DebugPlainPrint0(OUTER_LIMIT_MIN);
  DebugPlainPrint0(",");
  DebugPlainPrint0(OUTER_LIMIT_MAX);
  DebugPlainPrint0(" ORIGIN_LIMIT_MIN/MAX(mappedMin/Max):");
  DebugPlainPrint0(ORIGIN_LIMIT_MIN);
  DebugPlainPrint0(",");
  DebugPlainPrint0(ORIGIN_LIMIT_MAX);
  DebugPlainPrint0("(");
  DebugPlainPrint0(calcValues(ORIGIN_LIMIT_MAX, OUTER_LIMIT_MIN, OUTER_LIMIT_MAX).mapped);
  DebugPlainPrint0(",");
  DebugPlainPrint0(calcValues(ORIGIN_LIMIT_MIN, OUTER_LIMIT_MIN, OUTER_LIMIT_MAX).mapped);
  DebugPlainPrint0(")");
  DebugPlainPrintln0("");
}

void loop() {
  static int16_t prevX        = 0;
  static int16_t prevY        = 0;
  static int16_t upperLimitX  = OUTER_LIMIT_MAX;
  static int16_t lowerLimitX  = OUTER_LIMIT_MIN;
  static int16_t upperLimitY  = OUTER_LIMIT_MAX;
  static int16_t lowerLimitY  = OUTER_LIMIT_MIN;
  static uint8_t prevBtnState = 0;

  const int16_t rawX     = (int16_t)(AD_VALUE_MASK & analogRead(JX_PIN));
  const int16_t rawY     = (int16_t)(AD_VALUE_MASK & analogRead(JY_PIN));
  const uint8_t btnState = digitalRead(BUTTON_PIN);

  int16_t centerCalX = rawX + (MID_AD_VALUE - centerX);
  int16_t centerCalY = rawY + (MID_AD_VALUE - centerY);

  // Update limit values
  if (centerCalX - MARGIN_TO_UPDATE > upperLimitX) {
    upperLimitX = centerCalX - MARGIN_TO_UPDATE;
  }
  if (centerCalX + MARGIN_TO_UPDATE < lowerLimitX) {
    lowerLimitX = centerCalX + MARGIN_TO_UPDATE;
  }
  if (centerCalY - MARGIN_TO_UPDATE > upperLimitY) {
    upperLimitY = centerCalY - MARGIN_TO_UPDATE;
  }
  if (centerCalY + MARGIN_TO_UPDATE < lowerLimitY) {
    lowerLimitY = centerCalY + MARGIN_TO_UPDATE;
  }

  calcValues_t x = calcValues(centerCalX, lowerLimitX, upperLimitX);
  calcValues_t y = calcValues(centerCalY, lowerLimitY, upperLimitY);

  // Send updated values to core
  if (btnState != prevBtnState) {
    prevBtnState = btnState;
    Vivicore.write(1, btnState ? false : true); // 1: PUSH BUTTON
    DebugPlainPrint0(btnState);
    DebugPlainPrint0(", ");
  }

  if (y.calibrated != prevY) {
    prevY = y.calibrated;
    Vivicore.write(2, y.mapped); // 2: Up & Down
  }

  if (x.calibrated != prevX) {
    prevX = x.calibrated;
    Vivicore.write(3, x.mapped); // 3: Left & Right
  }

  Vivicore.flush();

  DebugPlainPrint0("raw:");
  DebugPlainPrint0(rawX);
  DebugPlainPrint0(",");
  DebugPlainPrint0(rawY);
  DebugPlainPrint0(" lowerLimit:");
  DebugPlainPrint0(lowerLimitX);
  DebugPlainPrint0(",");
  DebugPlainPrint0(lowerLimitY);
  DebugPlainPrint0(" upperLimit:");
  DebugPlainPrint0(upperLimitX);
  DebugPlainPrint0(",");
  DebugPlainPrint0(upperLimitY);
  DebugPlainPrint0(" calibrated:");
  DebugPlainPrint0(x.calibrated);
  DebugPlainPrint0(",");
  DebugPlainPrint0(y.calibrated);
  DebugPlainPrint0(" mapped:");
  DebugPlainPrint0(x.mapped);
  DebugPlainPrint0(",");
  DebugPlainPrint0(y.mapped);
  DebugPlainPrint0(" mappedh:");
  DebugHexPrint0(highByte(x.mapped));
  DebugHexPrint0(lowByte(x.mapped));
  DebugPlainPrint0(",");
  DebugHexPrint0(highByte(y.mapped));
  DebugHexPrint0(lowByte(y.mapped));
  DebugPlainPrintln0("");

  delay(10);
}
