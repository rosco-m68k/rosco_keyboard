#ifndef __ROSCO_KEYBOARD_COMMAND_H
#define __ROSCO_KEYBOARD_COMMAND_H

#include "config.hpp"

// Commands
#define CMD_LED_POWRED      0x1
#define CMD_LED_POWGRN      0x2
#define CMD_LED_POWBLU      0x3
#define CMD_LED_CAPS        0x4
#define CMD_LED_DISK        0x5
#define CMD_LED_EXTRED      0x6
#define CMD_LED_EXTGRN      0x7
#define CMD_LED_EXTBLU      0x8
// 0x9 - 0xf reserved...
#define CMD_MODE_SET        0x10
#define CMD_RPT_DELAY_SET   0x11
#define CMD_RPT_RATE_SET    0x12
// 0x13 - 0x1f reserved...
#define CMD_MOUSE_DETECT    0x20
#define CMD_MOUSE_STRM_ON   0x21
#define CMD_MOUSE_STRM_OFF  0x22
#define CMD_MOUSE_REPORT    0x23
#define CMD_MOUSE_SET_RATE  0x24
#define CMD_MOUSE_SET_RES   0x25
#define CMD_MOUSE_SET_SCALE 0x26
#ifdef REVISION_2
// 0x23 - 0x2f reserved...
#define CMD_SPI_ENABLE      0x30
#define CMD_SPI_DISABLE     0x31
#endif
// 0x32 - 0xef reserved...
#define CMD_IDENT           0xf0
#define CMD_RESET           0xf1
// 0xf2 - 0xff reserved...

#define CMD_ACK             ((uint8_t)0xff)
#define CMD_NAK             ((uint8_t)0x0)

// Modes 2..127 reserved, 128+ allowed for third-party modes
#define CMD_MODE_SCAN       ((0))
#define CMD_MODE_ASCII      ((1))

#define CMD_MOUSE_SCL_11    0x01
#define CMD_MOUSE_SCL_21    0x02

#define KEY_COUNT           ((uint8_t)67)
#define LED_COUNT           ((uint8_t)8)

#define CAP_KBD             0x01
#define CAP_SPI             0x02
#define CAP_I2C             0x04
#define CAP_PWM             0x08
#define CAP_PS2             0x10
#define CAP_RESERVED        0x80    // Must never be set!

// For these, I2C or PS2 are added later depending on jumper config...
#ifdef REVISION_2
#define CAPABILITIES        ((uint8_t)(CAP_KBD | CAP_SPI | CAP_PWM))
#else
#define CAPABILITIES        ((uint8_t)(CAP_KBD | CAP_SPI))
#endif

#define IDENT_MODE_SCAN     ((uint8_t)0)
#define IDENT_MODE_ASCII    ((uint8_t)1)

#endif//__ROSCO_KEYBOARD_COMMAND_H
