@echo off
git pull
platformio run --target clean --environment ATmega64
platformio run --environment ATmega64
avrdude -c usbtiny -p atmega64a -U flash:w:.pio\build\ATmega64\firmware.hex:i -U hfuse:w:0xcf:m -U lfuse:w:0xbf:m -U efuse:w:0xff:m -U lock:w:0xc0:m
