#define MIN_LIBRARY_VER_BUILD_NO (0x0005)
#include <VivicoreSerial.h>
#include <Wire.h>
#include <misakiUTF16.h>
#include "31FL3731regs.h"

#define EN_PIN   (10)
#define IRQ1_PIN (2)
#define IRQ2_PIN (3)

#define LEDX         (24)
#define LEDX_BY_IC   (16)
#define LEDX_BY_CHAR (4)
#define LEDY         (9)

#define SCROLL_INTERVAL  (70)   // 100: 100mS per line
#define BRIGHT_NORMAL    (0x33) // 1:min, 0xff:max 2mA per led chip.
#define BRIGHT_LOW       (0x10) // 1:min, 0xff:max 2mA per led chip.
#define MAX_BURST_LENGTH (31)

#define UTF16_CHAR_PERIOD (0xFF61) // Hankaku Kana min value
#define UTF16_CHAR_PSOUND (0xFF9F) // Hankaku Kana max value

#define DISP_BUF_SIZE (17)
#define WORK_BUF_SIZE (DISP_BUF_SIZE + 1) // String data size 17 + setting flag 1
#define FONT_V_SIZE   (FONT_LEN)
#define IMAGE_H_NUM   (DISP_BUF_SIZE + 2)
#define IMAGE_H_SIZE  (IMAGE_H_NUM * 4)
#define IMAGE_V_SIZE  (max(LEDY, FONT_V_SIZE))
#define IMAGE_V_SHIFT (-1)

const char *INITIAL_TEXT = "";

const uint16_t USER_FW_VER = 0x000E;
const uint32_t BRANCH_TYPE = 0x00000009;

const dcInfo_t dcInfo[] = {
  // {group_no, data_nature, data_type, data_min, data_max}
  {DC_GROUP_1, DC_NATURE_IN, DC_TYPE_BINARY, 0, 255} // 1: Text Input
};

const uint8_t slave_adrs[2] = {0x77, 0x74};
const uint8_t connected_banks[2][2] = {
  // CA, CB
  { 1, 1 }, // slave_adrs[0]
  { 0, 1 }, // slave_adrs[1]
};
byte ImageBuf[IMAGE_H_NUM][IMAGE_V_SIZE] = {}; // IMAGE_H_NUM 文字 x IMAGE_V_SIZE byte pattern buffer
byte ImageWidth[IMAGE_H_NUM] = {};             // font width
byte ImageBright[IMAGE_H_NUM] = {};            // Bright values
byte ImageLen = 0;                             // 文字数

inline void writeReg(const uint8_t slave_adr, const uint8_t reg, const uint8_t data) {
  Wire.beginTransmission(slave_adr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void selectPage(const uint8_t slave_adr, const uint8_t page) {
  writeReg(slave_adr, REG_POINT_TO_PAGE, page);
}

void writeRegByPage(const uint8_t slave_adr, const uint8_t page, const uint8_t reg, const uint8_t data) {
  selectPage(slave_adr, page);
  writeReg(slave_adr, reg, data);
}

void writeRegBurst(const uint8_t slave_adr, const uint8_t reg, const int len, const byte data) {
  // maximum burst length is 31! there is no description in datasheet.
  if (len > MAX_BURST_LENGTH) {
    return;
  }
  Wire.beginTransmission(slave_adr);
  Wire.write(reg);
  for (int i = 0; i < len; i++) {
    Wire.write(data);
  }
  Wire.endTransmission();
}

inline uint8_t convPosToSlaveAdr(const uint8_t x) {
  const uint8_t idx = (x < LEDX_BY_IC) ? 0 : 1;
  return slave_adrs[idx];
}

inline uint8_t convPosToPwmReg(const uint8_t x, const uint8_t y) {
  const uint8_t x_ = (x < LEDX_BY_IC) ? x : x - LEDX_BY_IC;
  return REG_START_PWM_CTRL + (LEDX_BY_IC - x_) - 1 + (y * LEDX_BY_IC);
}

// draw a pixel on the screen. The left up side is origin.
// illum is 0x40 is good in the bright room.
void writeDot(const int x, const int y, const int illum) {
  if ((x >= LEDX) || (x < 0) || (y >= LEDY) || (y < 0)) {
    return;
  }
  writeReg(convPosToSlaveAdr(x), convPosToPwmReg(x, y), illum);
}

// Clear screen with using burst command.
void fillScreen(const uint8_t illum) {
  for (uint8_t idx = 0; idx < sizeof(slave_adrs); idx++) {
    const uint8_t num_burst = (REG_END_PWM_CTRL - REG_START_PWM_CTRL + 1) / MAX_BURST_LENGTH;
    const uint8_t last_burst_size = (REG_END_PWM_CTRL - REG_START_PWM_CTRL + 1) % MAX_BURST_LENGTH;
    for (uint8_t n = 0; n < num_burst; n++) {
      writeRegBurst(slave_adrs[idx], REG_START_PWM_CTRL + n * MAX_BURST_LENGTH, MAX_BURST_LENGTH, illum);
    }
    writeRegBurst(slave_adrs[idx], REG_START_PWM_CTRL + num_burst * MAX_BURST_LENGTH, last_burst_size, illum);
  }
}

void writeImage(const char *pUTF8) {
  static uint16_t pUTF16[IMAGE_H_NUM];
  uint8_t n = Utf8ToUtf16(pUTF16, const_cast<char *>(pUTF8));

  if (n > IMAGE_H_NUM) {
    n = IMAGE_H_NUM;
  }

  for (byte i = 0; i < n; i++) {
    const uint16_t d = pUTF16[i];
    bool validFont = false;

    ImageBright[i] = BRIGHT_NORMAL;
    memset(&ImageBuf[i][0], 0, IMAGE_V_SIZE);
    validFont = getFontDataByUTF16(&ImageBuf[i][0], pUTF16[i]);

    if (!validFont) {
      memset(&ImageBuf[i][0], 0xFF, IMAGE_V_SIZE);
      ImageWidth[i] = 4;
      ImageBright[i] = BRIGHT_LOW;
    } else if ((highByte(d) == 0) || ((d >= UTF16_CHAR_PERIOD) && (d <= UTF16_CHAR_PSOUND))) {
      ImageWidth[i] = 4;
    } else {
      ImageWidth[i] = 8;
    }
  }

  ImageLen = n;
  for (byte img_n = ImageLen; img_n < IMAGE_H_NUM; img_n++) {
    for (byte img_v = 0; img_v < IMAGE_V_SIZE; img_v++) {
      ImageBuf[img_n][img_v] = 0;
    }
    ImageWidth[img_n] = 0;
  }
}

void updateScr(const int shift_h, const int shift_v, const bool clear) {
  int x_lside = 0;

  if (clear) {
    fillScreen(0);
  }

  for (byte img_n = 0; img_n < ImageLen; img_n++) {
    const byte img_width = ImageWidth[img_n];
    const byte img_bright = ImageBright[img_n];

    for (byte img_v = 0; img_v < IMAGE_V_SIZE; img_v++) {
      const byte img_line = ImageBuf[img_n][img_v];

      for (byte img_h = 0; img_h < img_width; img_h++) {
        const uint8_t is_draw = bitRead(img_line, 7 - img_h);

        if (is_draw) {
          int x_scr = x_lside + img_h - shift_h;
          int y_scr = img_v - shift_v;
          DebugPlainPrint0(x_scr);
          DebugPlainPrint0(",");
          DebugPlainPrintln0(y_scr);
          if (x_scr < 0) {
            x_scr += IMAGE_H_NUM * 4;
          } else if (x_scr >= (IMAGE_H_NUM * 4)) {
            x_scr -= IMAGE_H_NUM * 4;
          }
          if (y_scr < 0) {
            y_scr += IMAGE_V_SIZE;
          } else if (y_scr >= IMAGE_V_SIZE) {
            y_scr -= IMAGE_V_SIZE;
          }
          DebugPlainPrint0(x_scr);
          DebugPlainPrint0(",");
          DebugPlainPrintln0(y_scr);
          writeDot(x_scr, y_scr, img_bright);
          // DebugPlainPrint0("*");
        } else {
          // DebugPlainPrint0(" ");
        }
      }
    }
    x_lside += img_width;
    DebugPlainPrintln0("");
  }
  DebugPlainPrintln0("");
}

void convCharImage(const uint8_t *charbuf) {
  char buf[DISP_BUF_SIZE + 1] = {}; // +1 is for null terminate
  memcpy(buf, charbuf, DISP_BUF_SIZE);
  writeImage(buf);
}

bool isValidCtrlReg(const uint8_t slave_adr, const uint8_t reg) {
  // LEDs which are no connected must be off by LED Control Register (Frame Registers) or it will affect other LEDs. 
  uint8_t adr_idx = 0;
  const uint8_t bank_idx = bitRead(reg, 0);

  for (; adr_idx < sizeof(slave_adrs); adr_idx++ ) {
    if (slave_adr == slave_adrs[adr_idx]) {
      break;
    }
  }
  if ((adr_idx >= sizeof(slave_adrs)) ||
      (REG_END_LED_CTRL < reg)) {
    return false;
  }

  return (bool)connected_banks[adr_idx][bank_idx];
}

void initDisp(void) {
  // Clear frame buffer before switching power to prevent showing garbage
  fillScreen(0);

  for (uint8_t n = 0; n < sizeof(slave_adrs); n++) {
    const uint8_t slave_adr = slave_adrs[n];
    uint8_t frame_reg;

    // shutdown->normal operation
    writeRegByPage(slave_adr, PAGE_TO_FUNC, FUNC_TO_SHUTDOWN, 0);
    delay(10);
    writeRegByPage(slave_adr, PAGE_TO_FUNC, FUNC_TO_SHUTDOWN, 1);
    // picture mode
    writeRegByPage(slave_adr, PAGE_TO_FUNC, FUNC_TO_CONFIG, 0);
    // display picture frame 0 only
    writeRegByPage(slave_adr, PAGE_TO_FUNC, FUNC_TO_PICT_DISP, 0);

    /// LED pixel pattern
    for (frame_reg = 0; frame_reg <= REG_END_LED_CTRL; frame_reg++) {
      if (isValidCtrlReg(slave_adr, frame_reg)) {
        writeRegByPage(slave_adr, 0, frame_reg, 0xff);
      } else {
        writeRegByPage(slave_adr, 0, frame_reg, 0x00);
      }
    }
    /// blink all off
    for (; frame_reg <= REG_END_BLINK_CTRL; frame_reg++) {
      writeRegByPage(slave_adr, 0, frame_reg, 0x00);
    }
    /// PWM all off
    for (; frame_reg <= REG_END_PWM_CTRL; frame_reg++) {
      writeRegByPage(slave_adr, 0, frame_reg, 0x00);
    }
    writeRegByPage(slave_adr, PAGE_TO_FUNC, FUNC_TO_AUDIO_SYNC, 0);  // audio sync off
    selectPage(slave_adr, 0);
  }
}

void initScr() {
  fillScreen(0);
  writeImage(INITIAL_TEXT);
  updateScr(0, IMAGE_V_SHIFT, false);
  delay(100);
}

void setup() {
  Vivicore.begin(BRANCH_TYPE, USER_FW_VER, dcInfo, countof(dcInfo),
                 MIN_LIBRARY_VER_BUILD_NO);

  digitalWrite(EN_PIN, HIGH);
  pinMode(EN_PIN, OUTPUT);
  pinMode(IRQ1_PIN, INPUT);
  pinMode(IRQ2_PIN, INPUT);

  Wire.begin();
  Wire.setClock(400000L); // 400kHz I2C clock

  // A basic scanner, see if it ACK's
  for (uint8_t n = 0; n < sizeof(slave_adrs); n++) {
    Wire.beginTransmission(slave_adrs[n]);
    if (Wire.endTransmission() != 0) {
      DebugPlainPrint1("No ack from 0x");
      DebugHexPrint1(slave_adrs[n]);
      DebugPlainPrintln1();
    }
  }

  initDisp();
  initScr();
}

void loop() {
  static uint8_t statbuffer[WORK_BUF_SIZE] = {};
  static uint8_t charbuffer[DISP_BUF_SIZE] = {};
  static bool is_scroll = 0;
  static int image_h_shift = 0;
  static unsigned long prev_time = 0;
  uint8_t recv_cnt = Vivicore.available();

  if (recv_cnt > 0) {
    DebugPlainPrint1("CNT:");
    DebugPlainPrintln1(recv_cnt);
    if (recv_cnt <= WORK_BUF_SIZE) {
      DebugPlainPrint1("Read: ");
      for (int i = 0; i < recv_cnt; i++) {
        statbuffer[i] = Vivicore.read();
        DebugHexPrint1(statbuffer[i]);
      }
    } else {
      DebugPlainPrint0("DATA too long");
      for (int i = 0; i < recv_cnt; i++) {
        Vivicore.read();
      }
      recv_cnt = 0;
    }
    DebugPlainPrintln0();

    is_scroll = ((statbuffer[0] & 0x01) != 0);
    for (uint8_t char_idx = 0; char_idx < DISP_BUF_SIZE; char_idx++) {
      charbuffer[char_idx] = (char_idx < recv_cnt) ? statbuffer[char_idx + 1] : ' ';
    }

    image_h_shift = 0;
    recv_cnt = 0;
    prev_time = millis();

    convCharImage(charbuffer);
    updateScr(image_h_shift, IMAGE_V_SHIFT, true);
  }

  // Dont care about millis overflows https://garretlab.web.fc2.com/arduino/lab/millis/
  if (is_scroll && (millis() - prev_time > SCROLL_INTERVAL)) {
    updateScr(image_h_shift, IMAGE_V_SHIFT, true);
    image_h_shift++;
    prev_time += SCROLL_INTERVAL;
    if (image_h_shift > IMAGE_H_SIZE) {
      image_h_shift = 0;
    }
    DebugPlainPrintln0(image_h_shift);
  }
  // Vivicore.flush();

  delay(10);
}
