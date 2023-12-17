/*
 * Copyright (c) 2020 You (you@youremail.com)
 */

#include <stdio.h>
#include <stdint.h>
#include <buffered_uart.h>

#define CMD_IDENT           ((uint8_t)0xf0)
#define CMD_ACK             ((uint8_t)0xff)
#define IDENT_MODE_ASCII    ((uint8_t)0x01)

// static RingBuffer uart_b;
static CharDevice device;
uint8_t buf;
volatile long * ticks = (volatile long*)0x40c;
uint8_t step;

int tryGetChar(CharDevice *device) {
    long end = *ticks + 20;

    while (*ticks < end) {
        if (mcCheckDevice(device)) {
            return (uint8_t)mcReadDevice(device);
        }
    }

    return -1;
}

bool detectKeyboard(CharDevice *device) {
    step = 0;

    // Clear buffer
    while (mcCheckDevice(device)) {
        mcReadDevice(device);
    }

    // Send "IDENT" command
    mcSendDevice(CMD_IDENT, device);

    int chr = tryGetChar(device);
    if (chr != 0x72) {
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr != 'o') {
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr != 's') {
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr != 'c') {
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr != 'o') {
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr != '_') {
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr != 'k') {
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr != 'b') {
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr != 'd') {
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr != IDENT_MODE_ASCII) {
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr == -1) {    // key count
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr == -1) {    // led count
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr == -1) {    // capabilities
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr != 0) {     // reserved - must be 0
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr != 0) {     // reserved2 - must be 0
        return false;
    }

    step++;

    chr = tryGetChar(device);
    if (chr != CMD_ACK) {   // required ack
        return false;
    }

    return true;
}

void kmain() {
    printf("Detect Keyboard\n");

    if (!mcCheckDeviceSupport()) {
        printf("ERROR: No device support\n");
        return;
    }

    if (mcGetDeviceCount() < 2) {
        printf("ERROR: Insufficient devices\n");
        return;
    }

    if (!mcGetDevice(1, &device)) {
        printf("ERROR: Unable to get device\n");
        return;
    }

    if (!detectKeyboard(&device)) {
        printf("ERROR: No keyboard found [step: %d]\n", step);
        return;
    }

#ifdef INTERRUPT_DRIVEN
    int n;
    duart_install_interrupt(&device, NULL, &uart_b);
#endif
    printf("Righto, type away...\n");

    while (true) {
#ifdef INTERRUPT_DRIVEN
        n = duart_unbuffer_one(&uart_b, &buf);
        if (n) {
#else
            buf = mcReadDevice(&device);
#endif
            switch (buf) {
            case 0x08:
                // backspace
                mcPrintchar(0x08);
                mcPrintchar(0x20);
                mcPrintchar(0x08);
                break;
            case 0x0a:
            case 0x0d:
                mcPrintchar(0x0a);
                mcPrintchar(0x0d);
                break;
            default:
                mcPrintchar(buf);
            }

#ifdef INTERRUPT_DRIVEN    
        }
#endif
    }
}

