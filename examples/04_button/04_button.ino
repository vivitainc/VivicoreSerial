#define MIN_LIBRARY_VER_BUILD_NO (0x0012)
#include <VivicoreSerial.h>

#define BUTTON_NUM (4)
#define UP_PIN     (2)
#define LEFT_PIN   (3)
#define RIGHT_PIN  (9)
#define DOWN_PIN   (10)

const uint16_t USER_FW_VER = 0x000A;
const uint32_t BRANCH_TYPE = 0x00000004;

const int buttonPins[BUTTON_NUM] = {
  UP_PIN,    // 1: Button UP (X)
  LEFT_PIN,  // 2: Button LEFT (Y)
  RIGHT_PIN, // 3: Button RIGHT (A)
  DOWN_PIN,  // 4: Button DOWN (B)
};

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_BOOLEAN, 0, 1}, // 1: Button UP (X)
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_BOOLEAN, 0, 1}, // 2: Button LEFT (Y)
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_BOOLEAN, 0, 1}, // 3: Button RIGHT (A)
  {DcGroup_t::DC_GROUP_1, DcNature_t::DC_NATURE_OUT, DcType_t::DC_TYPE_BOOLEAN, 0, 1}, // 4: Button DOWN (B)
};

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo), MIN_LIBRARY_VER_BUILD_NO);
  for (int i = 0; i < BUTTON_NUM; i++) {
    pinMode(buttonPins[i], INPUT);
  }
}

void loop() {
  static uint8_t prevButtonState[BUTTON_NUM] = {};
  uint8_t        buttonState[BUTTON_NUM]     = {};

  delay(10);

  for (uint8_t buttonIndex = 0; buttonIndex < BUTTON_NUM; buttonIndex++) {
    buttonState[buttonIndex] = digitalRead(buttonPins[buttonIndex]);

    if (buttonState[buttonIndex] != prevButtonState[buttonIndex]) {
      prevButtonState[buttonIndex] = buttonState[buttonIndex];
      Vivicore.write(buttonIndex + 1, buttonState[buttonIndex] != 0);
    }
  }

  Vivicore.flush();
}
