#ifndef __ROSCO_KEYBOARD_CONFIG_H
#define __ROSCO_KEYBOARD_CONFIG_H

#define ROW_COUNT           5
#define COL_COUNT           15

#define LED_OPORT           PORTG
#define LED_DDR             DDRG

#define POW_OPORT           PORTE
#define POW_DDR             DDRE

#define POW_RED             0x08
#define POW_GRN             0x10
#define POW_BLU             0x20

#define LED_CAPS            0x01
#define LED_DISK            0x02
#define LED_EXTRED          0x04
#define LED_EXTGRN          0x08
#define LED_EXTBLU          0x10

#define DEBOUNCE_MILLIS     10
#define ROW_SETTLE_DELAY    0
#define DEFAULT_RPT_DELAY   500
#define DEFAULT_RATE_LIMIT  50

#define SPI_BUF_SIZE        0x200
#define SPI_BUF_MASK        ((SPI_BUF_SIZE-1))

#define PS2_CLOCK           PIN_PD0
#define PS2_DATA            PIN_PD1
#define PS2_MOUSE_BTN_LEFT  0
#define PS2_MOUSE_BTN_MID   0
#define PS2_MOUSE_BTN_RIGHT 0
#define PS2_PKT_START_CODE  0x60            // Scancode: Not down, row 6, col 0

#define CAPS_WORD_FLASH_D   300

#define CAPS_CODE_U         0x31
#define SHIFT_LEFT_CODE_U   0x41
#define SHIFT_RIGHT_CODE_U  0x4d
#define CTRL_LEFT_CODE_U    0x51
#define CTRL_RIGHT_CODE_U   0x5e
#define CMD_LEFT_CODE_U     0x52
#define CMD_RIGHT_CODE_U    0x5c
#define ROSCO_LEFT_CODE_U   0x53
#define ROSCO_RIGHT_CODE_U  0x5b
#define OPTION_CODE_U       0x5d

#define CAPS_CODE_D         ((CAPS_CODE_U         | 0x80))
#define SHIFT_LEFT_CODE_D   ((SHIFT_LEFT_CODE_U   | 0x80))
#define SHIFT_RIGHT_CODE_D  ((SHIFT_RIGHT_CODE_U  | 0x80))
#define CTRL_LEFT_CODE_D    ((CTRL_LEFT_CODE_U    | 0x80))
#define CTRL_RIGHT_CODE_D   ((CTRL_RIGHT_CODE_U   | 0x80))
#define CMD_LEFT_CODE_D     ((CMD_LEFT_CODE_U     | 0x80))
#define CMD_RIGHT_CODE_D    ((CMD_RIGHT_CODE_U    | 0x80))
#define ROSCO_LEFT_CODE_D   ((ROSCO_LEFT_CODE_U   | 0x80))
#define ROSCO_RIGHT_CODE_D  ((ROSCO_RIGHT_CODE_U  | 0x80))
#define OPTION_CODE_D       ((OPTION_CODE_U       | 0x80))

#endif//__ROSCO_KEYBOARD_CONFIG_H
