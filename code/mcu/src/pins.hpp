#ifndef __ROSCO_KEYBOARD_PINS_H
#define __ROSCO_KEYBOARD_PINS_H

#include <avr/pgmspace.h>
#include <pins_arduino.h>
#include "config.hpp"

#define UART_SPEED_JP_I     PIN_PF6         // JP1
#define UART_MODE_JP_I      PIN_PF7         // JP2
#define PASS_THRU_JP_I      PIN_PF4         // JP3
#define I2C_JP_I            PIN_PF3         // JP4
#define JP5_IN              PIN_PF2         // JP5
#define G_DISABLE_JP_I      PIN_PF1         // JP6

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

#endif//__ROSCO_KEYBOARD_PINS_H
