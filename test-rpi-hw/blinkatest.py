#!/usr/bin/env python3
# coding = utf-8
# Test/Diagnose Adafruit "Blinka" interface on a Raspberry Pi / Ubuntu
import board
import digitalio
import busio

print("Hello blinka!")

# Try to create a Digital input
pin = digitalio.DigitalInOut(board.D4)
print("Digital IO ok!")

# Try to create an I2C device
i2c = busio.I2C(board.SCL, board.SDA)
print("I2C ok!")

# Try to create an SPI device
spi = busio.SPI(board.SCLK, board.MOSI, board.MISO)
print("SPI ok!")

print("done!")