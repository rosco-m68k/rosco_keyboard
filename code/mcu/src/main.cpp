/*
 * rosco_m68k keyboard microcontroller code
 *
 * Copyright (c)2023 The Really Old-School Company Limited
 *
 * Closed source; Not for distribution.
 * 
 * Scancode format:
 *
 *     Down? | Row                   | Column
 *     ------|-----------------------|--------------------------------
 *     1     | 1     | 1     | 1     | 1     | 1     | 1     | 1
 *     ------|-----------------------|--------------------------------
 *     128   |  64   | 32    | 16    | 8     | 4     | 2     | 1
 * 
 * REVISION TWO:
 * 
 * Each byte received via the SPI interface will result in two scancode 
 * bytes, both identified as UP, column 7, with the low nibble containing
 * half of the received byte, e.g. 0b0111nnnn.
 *
 * Row and column are 1-based in this scheme.
 */

#include <Arduino.h>
#include <SPI.h>
#include <avr/wdt.h>

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
#define CMD_SPI_SET         0x20
#define CMD_SPI_XFER        0x21
// 0x22 - 0xef reserved...
#define CMD_IDENT           0xf0
#define CMD_RESET           0xf1
// 0xf2 - 0xff reserved...

#define CMD_ACK             ((uint8_t)0xff)
#define CMD_NAK             ((uint8_t)0x0)

#define KEY_COUNT           ((uint8_t)67)
#define LED_COUNT           ((uint8_t)8)

#define CAP_KBD             0x01
#define CAP_SPI             0x02
#define CAP_I2C             0x04
#define CAP_PWM             0x08
#define CAP_RESERVED        0x80    // Must never be set!
#define CAPABILITIES        ((uint8_t)(CAP_KBD | CAP_SPI | CAP_I2C | CAP_PWM))

#define IDENT_MODE_SCAN     ((uint8_t)0)
#define IDENT_MODE_ASCII    ((uint8_t)1)

// On mega64 we're using UART 1 as the main
#define M_UART              Serial1
// ... and UART 0 as the passthru
#define PT_UART             Serial

#define UART_SPEED_JP_I     PIN_PF6         // JP1
#define UART_MODE_JP_I      PIN_PF7         // JP2
#define PASS_THRU_JP_I      PIN_PF4         // JP3
#define JP4_IN              PIN_PF3         // JP4
#define JP5_IN              PIN_PF2         // JP5
#define G_DISABLE_JP_I      PIN_PF1         // JP6

#define SPI_BUF_SIZE        0x200
#define SPI_BUF_MASK        ((SPI_BUF_SIZE-1))

const PROGMEM uint8_t row_pins[] = {
#ifdef REVISION_2
    PIN_PC5,
    PIN_PC4,
#else
    PIN_PB1,
    PIN_PB0,
#endif
    PIN_PA6,
    PIN_PA5,
    PIN_PA4,
};

const PROGMEM uint8_t col_pins[] = {
    PIN_PA0,
    PIN_PA1,
    PIN_PA2,
    PIN_PA3,
    PIN_PA7,
#ifdef REVISION_2
    PIN_PC6,
    PIN_PC7,
#else
    PIN_PB2,
    PIN_PB3,
#endif
    PIN_PB4,
    PIN_PB5,
    PIN_PB7,
    PIN_PB6,
    PIN_PC3,
    PIN_PC2,
    PIN_PC1,
    PIN_PC0,
};

const PROGMEM uint8_t uart_keys_nocaps_unshift[] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   27,  '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-', '=', 8,   0,
    0,   '\t','q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', '[', ']', '\\',0,
    0,   0,   'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';', '\'',0,   '\r',0,
    0,   0,   '`', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', 0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   ' ', 0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
};

const PROGMEM uint8_t uart_keys_nocaps_shift[] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   27,  '!', '@', '#', '$', '%', '^', '&', '*', '(', ')', '_', '+', 8,   0,
    0,   '\t','Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P', '{', '}', '|', 0,
    0,   0,   'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', ':', '"', 0,   '\r',0,
    0,   0,   '~', 'Z', 'X', 'C', 'V', 'B', 'N', 'M', '<', '>', '?', 0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   ' ', 0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
};

const PROGMEM uint8_t uart_keys_caps_unshift[] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   27,  '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-', '=', 8,   0,
    0,   '\t','Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P', '[', ']', '\\',0,
    0,   0,   'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', ';', '\'',0,   '\r',0,
    0,   0,   '`', 'Z', 'X', 'C', 'V', 'B', 'N', 'M', ',', '.', '/', 0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   ' ', 0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
};

const PROGMEM uint8_t uart_keys_caps_shift[] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   27,  '!', '@', '#', '$', '%', '^', '&', '*', '(', ')', '_', '+', 8,   0,
    0,   '\t','q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', '{', '}', '|', 0,
    0,   0,   'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ':', '"', 0,   '\r',0,
    0,   0,   '~', 'z', 'x', 'c', 'v', 'b', 'n', 'm', '<', '>', '?', 0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   ' ', 0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
};

const PROGMEM uint8_t uart_keys_control[] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   17,  23,  5,   18,  20,  25,  21, 9,   15,  16,  27,   0,   0,   0,
    0,   0,   1,   19,  4,   6,   7,   8,   10,  11,  12,  0,   0,   0,   '\r',0,
    0,   0,   0,   26, 24,   3,   22,  2,   14,  13,  0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
};

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

#define CAPS_CODE_D         CAPS_CODE_U         | 0x80
#define SHIFT_LEFT_CODE_D   SHIFT_LEFT_CODE_U   | 0x80
#define SHIFT_RIGHT_CODE_D  SHIFT_RIGHT_CODE_U  | 0x80
#define CTRL_LEFT_CODE_D    CTRL_LEFT_CODE_U    | 0x80
#define CTRL_RIGHT_CODE_D   CTRL_RIGHT_CODE_U   | 0x80
#define CMD_LEFT_CODE_D     CMD_LEFT_CODE_U     | 0x80
#define CMD_RIGHT_CODE_D    CMD_RIGHT_CODE_U    | 0x80
#define ROSCO_LEFT_CODE_D   ROSCO_LEFT_CODE_U   | 0x80
#define ROSCO_RIGHT_CODE_D  ROSCO_RIGHT_CODE_U  | 0x80
#define OPTION_CODE_D       OPTION_CODE_U       | 0x80

static bool keys[ROW_COUNT][COL_COUNT];
static unsigned long keys_t[ROW_COUNT][COL_COUNT];
static bool keys_r[ROW_COUNT][COL_COUNT];

static uint8_t current_command;
static uint8_t spi_set_next_byte;

static uint32_t spi_clock;
static uint8_t spi_mode;
static uint8_t spi_bitorder;

static uint16_t repeat_delay;
static uint8_t repeat_rate_limit;

static bool global_disable;
static bool uart_mode;
static bool uart_passthru;
static bool uart_caps;
static bool uart_lshift;
static bool uart_rshift;
static bool uart_lctrl;
static bool uart_rctrl;
static bool uart_lcmd;
static bool uart_rcmd;
static bool uart_option;
static bool uart_lrosco;
static bool uart_rrosco;

#ifdef REVISION_2
static uint8_t spi_buf[512];
volatile static uint16_t spi_ptr_r;
volatile static uint16_t spi_ptr_w;
#endif

static inline __attribute__((always_inline)) uint8_t keyup(int row, int col) {
    return (row + 1) << 4 | (col + 1);
}

static inline __attribute__((always_inline)) uint8_t keydown(int row, int col) {
    return 0x80 | keyup(row, col);
}

/* ** LEDs on LED Port ** */
static inline __attribute__((always_inline)) void led_off(const uint8_t led_mask) {
    LED_OPORT |= led_mask;
}

static inline __attribute__((always_inline)) void led_on(const uint8_t led_mask) {
    LED_OPORT &= ~(led_mask);
}

static inline __attribute__((always_inline)) void led_set(const uint8_t led_mask, uint8_t brightness) {
    if (brightness) {
        led_on(led_mask);
    } else {
        led_off(led_mask);
    }
}

/* ** LEDs on POW Port ** */
static inline __attribute__((always_inline)) void pow_off(const uint8_t led_mask) {
    POW_OPORT |= led_mask;
}

static inline __attribute__((always_inline)) void pow_on(const uint8_t led_mask) {
    POW_OPORT &= ~(led_mask);
}

static inline __attribute__((always_inline)) void pow_set(const uint8_t led_mask, uint8_t brightness) {
    if (brightness) {
        led_on(led_mask);
    } else {
        led_off(led_mask);
    }
}

static void process_command(int byte) {
    if (byte < 0) {
        return;
    }

    if (current_command == 0) {
        switch (byte) {
        // starting a command
        case CMD_LED_POWRED:
        case CMD_LED_POWGRN:
        case CMD_LED_POWBLU:
        case CMD_LED_CAPS:
        case CMD_LED_DISK:
        case CMD_LED_EXTRED:
        case CMD_LED_EXTGRN:
        case CMD_LED_EXTBLU:
        case CMD_MODE_SET:
        case CMD_RPT_DELAY_SET:
        case CMD_RPT_RATE_SET:
        case CMD_SPI_XFER:
            current_command = byte;
            M_UART.write(CMD_ACK);
            break;
        case CMD_SPI_SET:
            current_command = byte;
            spi_set_next_byte = 0;
            spi_clock = 0;
            M_UART.write(CMD_ACK);
            break;
        case CMD_RESET:
            M_UART.write(CMD_ACK);
            wdt_enable(WDTO_15MS);
            while (1);
        case CMD_IDENT:
            M_UART.write("rosco_kbd");
            if (uart_mode) {
                M_UART.write(IDENT_MODE_ASCII);
            } else {
                M_UART.write(IDENT_MODE_SCAN);
            }
            M_UART.write(KEY_COUNT);
            M_UART.write(LED_COUNT);
            M_UART.write(CAPABILITIES);
            M_UART.write((uint8_t)0);
            M_UART.write((uint8_t)0);
            M_UART.write(CMD_ACK);
        default:
            M_UART.write(CMD_NAK);
        }
    } else {
        // operand
        switch (current_command) {
        case CMD_LED_POWRED:
            pow_set(POW_RED, byte);
            M_UART.write(CMD_ACK);
            current_command = 0;
            break;
        case CMD_LED_POWGRN:
            pow_set(POW_GRN, byte);
            M_UART.write(CMD_ACK);
            current_command = 0;
            break;
        case CMD_LED_POWBLU:
            pow_set(POW_BLU, byte);
            M_UART.write(CMD_ACK);
            current_command = 0;
            break;
        case CMD_LED_CAPS:
            led_set(LED_CAPS, byte);
            M_UART.write(CMD_ACK);
            current_command = 0;
            break;
        case CMD_LED_DISK:
            led_set(LED_DISK, byte);
            M_UART.write(CMD_ACK);
            current_command = 0;
            break;
        case CMD_LED_EXTRED:
            led_set(LED_EXTRED, byte);
            M_UART.write(CMD_ACK);
            current_command = 0;
            break;
        case CMD_LED_EXTGRN:
            led_set(LED_EXTGRN, byte);
            M_UART.write(CMD_ACK);
            current_command = 0;
            break;
        case CMD_LED_EXTBLU:
            led_set(LED_EXTBLU, byte);
            M_UART.write(CMD_ACK);
            current_command = 0;
            break;
        case CMD_MODE_SET:
            if (byte == 0) {
                uart_mode = false;
                M_UART.write(CMD_ACK);
            } else {
                uart_mode = true;
                M_UART.write(CMD_ACK);
            }
            current_command = 0;
            break;
        case CMD_RPT_DELAY_SET:
            repeat_delay = byte * 10;
            M_UART.write(CMD_ACK);
            current_command = 0;
            break;
        case CMD_RPT_RATE_SET:
            if (byte == 0) {
                M_UART.write(CMD_NAK);
            } else {
                repeat_rate_limit = (uint8_t)(256 - byte);
                M_UART.write(CMD_ACK);
            }
            current_command = 0;
            break;            
        case CMD_SPI_SET:
            switch (spi_set_next_byte) {
            case 0:
                spi_clock |= (((uint32_t)byte) << 24);
                spi_set_next_byte++;
                break;
            case 1:
                spi_clock |= (((uint32_t)byte) << 16);
                spi_set_next_byte++;
                break;
            case 2:
                spi_clock |= (((uint32_t)byte) << 8);
                spi_set_next_byte++;
                break;
            case 3:
                spi_clock |= byte;
                spi_set_next_byte++;
                break;
            case 4:
                spi_bitorder = byte;
                spi_set_next_byte++;
                break;
            case 5:
                spi_mode = byte;
                M_UART.write(CMD_ACK);
                current_command = 0;
                break;
            default:
                M_UART.write(CMD_NAK);
                current_command = 0;
            }
            break;
        case CMD_SPI_XFER:
            SPI.beginTransaction(SPISettings(spi_clock, spi_bitorder, spi_mode));
            M_UART.write(SPI.transfer(byte));
            SPI.endTransaction();
            M_UART.write(CMD_ACK);
            current_command = 0;
        default:
            M_UART.write(CMD_NAK);
            current_command = 0;
        }
    }
}

static inline __attribute__((always_inline)) bool key_is_repeatable(uint8_t key) {
    switch (key) {
    case CAPS_CODE_U:
    case CAPS_CODE_D:
    case SHIFT_LEFT_CODE_D:
    case SHIFT_LEFT_CODE_U:
    case SHIFT_RIGHT_CODE_D:
    case SHIFT_RIGHT_CODE_U:
    case CTRL_LEFT_CODE_D:
    case CTRL_LEFT_CODE_U:
    case CTRL_RIGHT_CODE_D:
    case CTRL_RIGHT_CODE_U:
    case CMD_LEFT_CODE_D:
    case CMD_LEFT_CODE_U:
    case CMD_RIGHT_CODE_D:
    case CMD_RIGHT_CODE_U:
    case ROSCO_LEFT_CODE_D:
    case ROSCO_LEFT_CODE_U:
    case ROSCO_RIGHT_CODE_D:
    case ROSCO_RIGHT_CODE_U:
    case OPTION_CODE_D:
    case OPTION_CODE_U:
        return false;
    }

    return true;
}

static inline __attribute__((always_inline)) bool uart_process_special(uint8_t key) {
    switch (key) {
    case CAPS_CODE_U:
        if ((uart_caps = !uart_caps)) {
            led_set(LED_CAPS, 255);
        } else {
            led_set(LED_CAPS, 0);
        }

        return true;
    case SHIFT_LEFT_CODE_D:
        uart_lshift = true;
        return true;
    case SHIFT_LEFT_CODE_U:
        uart_lshift = false;
        return true;
    case SHIFT_RIGHT_CODE_D:
        uart_rshift = true;
        return true;
    case SHIFT_RIGHT_CODE_U:
        uart_rshift = false;
        return true;
    case CTRL_LEFT_CODE_D:
        uart_lctrl = true;
        return true;
    case CTRL_LEFT_CODE_U:
        uart_lctrl = false;
        return true;
    case CTRL_RIGHT_CODE_D:
        uart_rctrl = true;
        return true;
    case CTRL_RIGHT_CODE_U:
        uart_rctrl = false;
        return true;
    case CMD_LEFT_CODE_D:
        uart_lcmd = true;
        return true;
    case CMD_LEFT_CODE_U:
        uart_lcmd = false;
        return true;
    case CMD_RIGHT_CODE_D:
        uart_rcmd = true;
        return true;
    case CMD_RIGHT_CODE_U:
        uart_rcmd = false;
        return true;
    case ROSCO_LEFT_CODE_D:
        uart_lrosco = true;
        return true;
    case ROSCO_LEFT_CODE_U:
        uart_lrosco = false;
        return true;
    case ROSCO_RIGHT_CODE_D:
        uart_rrosco = true;
        return true;
    case ROSCO_RIGHT_CODE_U:
        uart_rrosco = false;
        return true;
    case OPTION_CODE_D:
        uart_option = false;
        return true;
    case OPTION_CODE_U:
        uart_option = false;
        return true;
    }

    return false;
}

static inline __attribute__((always_inline)) uint8_t uart_keybreak_code(uint8_t code) {
    if (uart_lctrl || uart_rctrl) {
        return pgm_read_byte_near(uart_keys_control + code);
    } else if (uart_caps) {
        if (uart_lshift || uart_rshift) {
            return pgm_read_byte_near(uart_keys_caps_shift + code);
        } else {
            return pgm_read_byte_near(uart_keys_caps_unshift + code);
        }
    } else {
        if (uart_lshift || uart_rshift) {
            return pgm_read_byte_near(uart_keys_nocaps_shift + code);
        } else {
            return pgm_read_byte_near(uart_keys_nocaps_unshift + code);
        }
    }

    return 0;
}

static inline __attribute__((always_inline)) void uart_repeat_keypress(int row, int col) {
    uint8_t code = keyup(row, col);

    if (key_is_repeatable(code)) {
        uint8_t ascii = uart_keybreak_code(code);

        if (ascii > 0) {
            M_UART.write(ascii);
        }
    }
}

#ifdef REVISION_2
/*
 * SPI ISR
 */
ISR(SPI_STC_vect) {
    unsigned char c = SPDR;

    spi_buf[spi_ptr_w++] = c;
    spi_ptr_w &= SPI_BUF_MASK;
}

static inline __attribute__((always_inline)) void service_spi(void) {
    // Only sending one byte per call rather than emptying the buffer.
    // Since this is called in the matrix scan loop, this means we 
    // will keep servicing SPI without letting it overrun the whole 
    // system...
    //
    // We don't need to disable interrupts because of the circular buffer,
    // it's possible (but unlikely) we can miss data if the buffer overruns - 
    // disabling interrupts wouldn't help - we'd just miss it altogether
    // in that case which isn't any better (I'd actually argue worse since 
    // probably more likely...)
    //
    if (!uart_mode && spi_ptr_r != spi_ptr_w) {
        unsigned char c = spi_buf[spi_ptr_r++];
        spi_ptr_r &= SPI_BUF_MASK;
        M_UART.write(0x7 | (c & 0xF0) >> 4);
        M_UART.write(0x7 | c & 0x0F);
    }
}
#else
#define service_spi(void)   0
#endif

static inline __attribute__((always_inline)) void uart_loop(void) {
    unsigned long now = millis();

    for (int row = 0; row < ROW_COUNT; row++) {
        uint8_t row_pin = pgm_read_byte_near(row_pins + row);
        pinMode(row_pin, OUTPUT);
        digitalWrite(row_pin, LOW);

#       if ROW_SETTLE_DELAY
        delayMicroseconds(ROW_SETTLE_DELAY);
#       endif

        for (int col = 0; col < COL_COUNT; col++) {
            // Handle any passthru...
            if (uart_passthru) {
                while (PT_UART.available() > 0) {
                    M_UART.write(PT_UART.read());
                }
                while (M_UART.available() > 0) {
                    PT_UART.write(M_UART.read());
                }
            }

            uint8_t col_pin = pgm_read_byte_near(col_pins + col);

            if (now - keys_t[row][col] > DEBOUNCE_MILLIS) {
                if (!digitalRead(col_pin)) {
                    if (!keys[row][col]) {
                        keys[row][col] = true;
                        keys_t[row][col] = now;

                        uart_process_special(keydown(row, col));
                    } else {
                        // key already down - are we repeating?
                        if (repeat_delay > 0) {
                            if (keys_r[row][col]) {
                                // Already repeating - time for another break?
                                if (now - keys_t[row][col] > repeat_rate_limit) {
                                    // yes - send it
                                    uart_repeat_keypress(row, col);
                                    keys_t[row][col] = now;
                                }
                            } else {
                                // not repeating - time to start?
                                if (now - keys_t[row][col] > repeat_delay) {
                                    // yes - start repeating
                                    uart_repeat_keypress(row, col);
                                    keys_t[row][col] = now;
                                    keys_r[row][col] = true;
                                }
                            }
                        }
                    }
                } else {
                    if (keys[row][col]) {
                        keys[row][col] = false;
                        keys_r[row][col] = false;
                        keys_t[row][col] = now;

                        uint8_t code = keyup(row, col);

                        if (!uart_process_special(code)) {
                            uint8_t ascii = uart_keybreak_code(code);

                            if (ascii > 0) {
                                M_UART.write(ascii);
                            }
                        }
                    }
                }
            }
        }

        digitalWrite(row_pin, HIGH);
        pinMode(row_pin, INPUT);
    }

    // Any commands waiting?
    if (!uart_passthru) {
        while (M_UART.available()) {
            process_command(M_UART.read());
        }
    }

    // Does SPI need anything?
    service_spi();
}

static inline __attribute__((always_inline)) void scancode_loop(void) {
    unsigned long now = millis();

    for (int row = 0; row < ROW_COUNT; row++) {
        uint8_t row_pin = pgm_read_byte_near(row_pins + row);
        pinMode(row_pin, OUTPUT);
        digitalWrite(row_pin, LOW);

#       if ROW_SETTLE_DELAY
        delayMicroseconds(ROW_SETTLE_DELAY);
#       endif

        for (int col = 0; col < COL_COUNT; col++) {
            // Handle any passthru...
            if (uart_passthru) {
                while (PT_UART.available() > 0) {
                    M_UART.write(PT_UART.read());
                }
                while (M_UART.available() > 0) {
                    PT_UART.write(M_UART.read());
                }
            }

            uint8_t col_pin = pgm_read_byte_near(col_pins + col);

            if (now - keys_t[row][col] > DEBOUNCE_MILLIS) {
                if (!digitalRead(col_pin)) {
                    if (!keys[row][col]) {
                        keys[row][col] = true;
                        keys_t[row][col] = now;
                        M_UART.write(keydown(row, col));
                    } else {
                        // key already down - are we repeating?
                        if (key_is_repeatable(keydown(row, col)) && repeat_delay > 0) {
                            if (keys_r[row][col]) {
                                // Already repeating - time for another break?
                                if (now - keys_t[row][col] > repeat_rate_limit) {
                                    // yes - send it
                                    M_UART.write(keyup(row, col));
                                    keys_t[row][col] = now;
                                }
                            } else {
                                // not repeating - time to start?
                                if (now - keys_t[row][col] > repeat_delay) {
                                    // yes - start repeating
                                    M_UART.write(keyup(row, col));
                                    keys_t[row][col] = now;
                                    keys_r[row][col] = true;
                                }
                            }
                        }

                    }
                } else {
                    if (keys[row][col]) {
                        keys[row][col] = false;
                        keys_r[row][col] = false;
                        keys_t[row][col] = now;
                        M_UART.write(keyup(row, col));
                    }
                }
            }
        }

        digitalWrite(row_pin, HIGH);
        pinMode(row_pin, INPUT);
    }

    // Any commands waiting?
    if (!uart_passthru) {
        while (M_UART.available()) {
            process_command(M_UART.read());
        }
    }

    // Does SPI need anything?
    service_spi();
}

void setup(void) {
    // First, check global disable - if disabled, we don't touch any other pins...
    pinMode(G_DISABLE_JP_I, INPUT_PULLUP);
    global_disable = digitalRead(G_DISABLE_JP_I) == LOW;

    if (!global_disable) {
        // LED port output, all off
        POW_DDR = 0xff;
        POW_OPORT = 0xff;
        LED_DDR = 0xff;
        LED_OPORT = 0xff;

        // Turn on power LED (green)
        pow_on(POW_GRN);
        
        // Figure out which mode we're in
        pinMode(UART_MODE_JP_I, INPUT_PULLUP);
        uart_mode = digitalRead(UART_MODE_JP_I) == HIGH;

        // Figure out which speed we want, and should we enable pass-thru
        pinMode(UART_SPEED_JP_I, INPUT_PULLUP);
        pinMode(PASS_THRU_JP_I, INPUT_PULLUP);
        if (digitalRead(UART_SPEED_JP_I) == LOW) {
            M_UART.begin(9600);

            if (digitalRead(PASS_THRU_JP_I) == LOW) {
                uart_passthru = true;
                PT_UART.begin(9600);
            }
        } else {
            M_UART.begin(115200);

            if (digitalRead(PASS_THRU_JP_I) == LOW) {
                uart_passthru = true;
                PT_UART.begin(115200);
            }
        }

#ifdef REVISION_2
        // Setup as SPI slave if not in UART mode
        if (!uart_mode) {
            M_UART.print("Setting up SPI...\n");
            pinMode(MISO, OUTPUT);
            SPCR |= _BV(SPE);
            SPCR |= _BV(SPIE);
        }
#endif

        // Setup key matrix array
        for (int r = 0; r < ROW_COUNT; r++) {
            for (int c = 0; c < COL_COUNT; c++) {
                keys[r][c] = 0;
                keys_t[r][c] = 0;
            }
        }

        // Set up row pins
        for (int i = 0; i < ROW_COUNT; i++) {
            uint8_t row_pin = pgm_read_byte_near(row_pins + i);
            pinMode(row_pin, OUTPUT);
            digitalWrite(row_pin, HIGH);
        }

        // Set up column pins
        for (int i = 0; i < COL_COUNT; i++) {
            pinMode(pgm_read_byte_near(col_pins + i), INPUT_PULLUP);
        }

        // Restart after 1S unresponsive...
        wdt_enable(WDTO_1S);

        // default repeat
        repeat_delay = DEFAULT_RPT_DELAY;
        repeat_rate_limit = DEFAULT_RATE_LIMIT;
    }
}

void loop(void) {
    if (!global_disable) {
        if (uart_mode) {
            uart_loop();
        } else {
            scancode_loop();
        }
    }

    wdt_reset();
}
