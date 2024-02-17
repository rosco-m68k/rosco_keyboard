#!/bin/sh
git pull
make
avrdude -c usbtiny -p atmega32u4 -U flash:w:rosco_m68k_default.hex:i -U hfuse:w:0xd8:m -U lfuse:w:0xff:m -U efuse:w:0xcb:m
