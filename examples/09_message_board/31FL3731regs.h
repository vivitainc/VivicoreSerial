#ifndef _31FL3731_REGS_H
#define _31FL3731_REGS_H

#define REG_POINT_TO_PAGE    (0xFD)
#define PAGE_TO_FUNC         (0x0B)
#define FUNC_TO_CONFIG       (0x00)
#define FUNC_TO_PICT_DISP    (0x01)
#define FUNC_TO_AUDIO_SYNC   (0x06)
#define FUNC_TO_SHUTDOWN     (0x0A)

#define REG_START_LED_CTRL   (0x00)  // Table 4 00h ~ 11h LED Control Register
#define REG_END_LED_CTRL     (0x11)  // Table 4 00h ~ 11h LED Control Register
#define REG_START_BLINK_CTRL (0x12)  // Table 5 12h ~ 23h Blink Control Register
#define REG_END_BLINK_CTRL   (0x23)  // Table 5 12h ~ 23h Blink Control Register
#define REG_START_PWM_CTRL   (0x24)  // Table 6 24h ~ B3h PWM Register
#define REG_END_PWM_CTRL     (0xB3)  // Table 6 24h ~ B3h PWM Register

#endif