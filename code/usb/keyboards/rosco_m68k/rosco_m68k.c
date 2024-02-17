#include "rosco_m68k.h"

void keyboard_pre_init_kb(void) {
    setPinOutput(PIN_POW_RED);
    setPinOutput(PIN_POW_GRN);
    setPinOutput(PIN_POW_BLU);

    writePinLow(PIN_POW_RED);
    writePinHigh(PIN_POW_GRN);
    writePinHigh(PIN_POW_BLU);
}

void matrix_init_kb(void) {
    writePinLow(PIN_POW_RED);
    writePinLow(PIN_POW_GRN);
    writePinHigh(PIN_POW_BLU);
}

void keyboard_post_init_kb(void) {
    writePinHigh(PIN_POW_RED);
    writePinLow(PIN_POW_GRN);
    writePinHigh(PIN_POW_BLU);
}
