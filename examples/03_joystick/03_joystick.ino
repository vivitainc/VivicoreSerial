#define MIN_LIBRARY_VER_BUILD_NO (0x0008)
#include <VivicoreSerial.h>

//#define CENTER_CALIBRATION

#define AD_VALUE_MASK  (0x3FF)
#define MAX_AD_VALUE   (AD_VALUE_MASK)
#define MID_AD_VALUE   (MAX_AD_VALUE / 2)
#define MIN_VALUE      (-1 * MID_AD_VALUE)
#define MAX_VALUE      (MID_AD_VALUE)

#define MARGIN_TO_UPDATE    (20)   // X/Y axis margin to be updated if exceeding this value
#define MARGIN_ORIGIN       (70)   // n=3, worst case, 45
#define MARGIN_OUTER        (232 + 75)  // n=3, worst case, 207, Avg delta b/w DVT and T2 = approx. 75
#define ORIGIN_LIMIT_MIN    (MID_AD_VALUE - MARGIN_ORIGIN)
#define ORIGIN_LIMIT_MAX    (MID_AD_VALUE + MARGIN_ORIGIN)
#define OUTER_LIMIT_MIN     (MARGIN_OUTER)
#define OUTER_LIMIT_MAX     (MAX_AD_VALUE - MARGIN_OUTER)

#define BUTTON_PIN (2)
#define JX_PIN     (A2)
#define JY_PIN     (A0)

const uint16_t USER_FW_VER = 0x000A;
const uint32_t BRANCH_TYPE = 0x00000003;

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DC_GROUP_1, DC_NATURE_OUT, DC_TYPE_BOOLEAN,       0,         1},         // 1: PUSH BUTTON
  {DC_GROUP_1, DC_NATURE_OUT, DC_TYPE_ANALOG_2BYTES, MIN_VALUE, MAX_VALUE}, // 2: Up & Down
  {DC_GROUP_1, DC_NATURE_OUT, DC_TYPE_ANALOG_2BYTES, MIN_VALUE, MAX_VALUE}, // 3: Left & Right
};
uint8_t statbuffer[] = {
  1, (uint8_t)false, // 1: PUSH BUTTON
  2, 0, 0,           // 2: Up & Down
  3, 0, 0,           // 3: Left & Right
};

int16_t centerX = MID_AD_VALUE;
int16_t centerY = MID_AD_VALUE;

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);
  pinMode(BUTTON_PIN, INPUT);

#ifdef CENTER_CALIBRATION
  centerX = (int16_t)analogRead(JX_PIN);
  centerY = (int16_t)analogRead(JY_PIN);
#endif
}

void loop() {
  static int16_t prevX = 0;
  static int16_t prevY = 0;
  static int16_t upperLimitX = OUTER_LIMIT_MAX;
  static int16_t lowerLimitX = OUTER_LIMIT_MIN;
  static int16_t upperLimitY = OUTER_LIMIT_MAX;
  static int16_t lowerLimitY = OUTER_LIMIT_MIN;
  static uint8_t prevBtnState = 0;

  const int16_t rawX = (int16_t)(AD_VALUE_MASK & analogRead(JX_PIN));
  const int16_t rawY = (int16_t)(AD_VALUE_MASK & analogRead(JY_PIN));
  const uint8_t btnState = digitalRead(BUTTON_PIN);

  int16_t calibratedX = rawX + (MID_AD_VALUE - centerX);
  int16_t calibratedY = rawY + (MID_AD_VALUE - centerY);
  int16_t mappedX = 0;
  int16_t mappedY = 0;
  bool isUpdated = false;
  int i = 0;

  // Update limit values
  if (calibratedX - MARGIN_TO_UPDATE > upperLimitX) {
    upperLimitX = calibratedX - MARGIN_TO_UPDATE;
  }
  if (calibratedX + MARGIN_TO_UPDATE < lowerLimitX) {
    lowerLimitX = calibratedX + MARGIN_TO_UPDATE;
  }
  if (calibratedY - MARGIN_TO_UPDATE > upperLimitY) {
    upperLimitY = calibratedY - MARGIN_TO_UPDATE;
  }
  if (calibratedY + MARGIN_TO_UPDATE < lowerLimitY) {
    lowerLimitY = calibratedY + MARGIN_TO_UPDATE;
  }

  // Trimming center & corner data
  calibratedX = constrain(calibratedX, lowerLimitX, upperLimitX);
  if ((ORIGIN_LIMIT_MIN < calibratedX) && (calibratedX < ORIGIN_LIMIT_MAX)) {
    calibratedX = MID_AD_VALUE;
  }
  calibratedY = constrain(calibratedY, lowerLimitY, upperLimitY);
  if ((ORIGIN_LIMIT_MIN < calibratedY) && (calibratedY < ORIGIN_LIMIT_MAX)) {
    calibratedY = MID_AD_VALUE;
  }

  // Map AD values to X/Y axis values
  if (calibratedX <= MID_AD_VALUE) {
    mappedX = map(calibratedX, lowerLimitX, MID_AD_VALUE, MAX_VALUE, 0);
  } else {
    mappedX = map(calibratedX, MID_AD_VALUE, upperLimitX, -1, MIN_VALUE);
  }
  if (calibratedY <= MID_AD_VALUE) {
    mappedY = map(calibratedY, lowerLimitY, MID_AD_VALUE, MAX_VALUE, 0);
  } else {
    mappedY = map(calibratedY, MID_AD_VALUE, upperLimitY, -1, MIN_VALUE);
  }

  // Send updated values to core
  if (btnState != prevBtnState) {
    isUpdated = true;
    prevBtnState = btnState;
    statbuffer[i++] = 1;
    statbuffer[i++] = (uint8_t)(btnState ? false : true);
    DebugPlainPrint0(btnState);
    DebugPlainPrint0(", ");
  }

  if (calibratedY != prevY) {
    isUpdated = true;
    statbuffer[i++] = 2;
    statbuffer[i++] = highByte(mappedY);
    statbuffer[i++] = lowByte(mappedY);
    prevY = calibratedY;
  }

  if (calibratedX != prevX) {
    isUpdated = true;
    statbuffer[i++] = 3;
    statbuffer[i++] = highByte(mappedX);
    statbuffer[i++] = lowByte(mappedX);
    prevX = calibratedX;
  }

  if (isUpdated) {
    Vivicore.write(statbuffer, i);
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
  DebugPlainPrint0(calibratedX);
  DebugPlainPrint0(",");
  DebugPlainPrint0(calibratedY);
  DebugPlainPrint0(" mapped:");
  DebugPlainPrint0(mappedX);
  DebugPlainPrint0(",");
  DebugPlainPrint0(mappedY);
  DebugPlainPrint0(" mappedh:");
  DebugHexPrint0(highByte(mappedX));
  DebugHexPrint0(lowByte(mappedX));
  DebugPlainPrint0(",");
  DebugHexPrint0(highByte(mappedY));
  DebugHexPrint0(lowByte(mappedY));
  DebugPlainPrintln0("");

  delay(10);
}
