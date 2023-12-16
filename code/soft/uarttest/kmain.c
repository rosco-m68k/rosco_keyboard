/*
 * Copyright (c) 2020 You (you@youremail.com)
 */

#include <stdio.h>
#include <stdint.h>
#include <buffered_uart.h>

static RingBuffer uart_b;
static CharDevice device;
uint8_t buf;

void kmain() {
    int i = 0;
    int n = 1;

    printf("Install handlers\n");

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

    duart_install_interrupt(&device, NULL, &uart_b);

    printf("Righto, that's done...\n");

    int led_on = 0;

    while (true) {
        if (i++ == 3) {
            mcSendDevice(0x5, &device);

            if (led_on) {
                mcSendDevice(0, &device);
                led_on = 0;
            } else {
                mcSendDevice((unsigned char)255, &device);
                led_on = 1;
            }
            i = 0;
        }

        buf = mcReadDevice(&device);
        // n = duart_unbuffer_one(&uart_b, &buf);

        if (n) {
            printf("%c", buf);
        }
    }
}

