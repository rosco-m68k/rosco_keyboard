#!/bin/sh
git pull
make -r -R -f builddefs/build_keyboard.mk KEYBOARD=rosco_m68k KEYMAP=default KEYBOARD_FILESAFE=rosco_m68k TARGET=rosco_m68k_default INTERMEDIATE_OUTPUT=.build/obj_rosco_m68k_default VERBOSE=true COLOR=true SILENT=false
avrdude -c usbtiny -p atmega32u4 -U flash:w:rosco_m68k_default.hex:i -U hfuse:w:0xd8:m -U lfuse:w:0xff:m -U efuse:w:0xcb:m
