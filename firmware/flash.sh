#!/bin/sh

echo "Press RESET on teensy2.0 board..."

./teensy_loader_cli --mcu=atmega32u4 -w alcor4.hex
