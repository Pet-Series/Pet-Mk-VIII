#!/bin/bash
# Test script for i2c access. Scan/Detect all available I2C-devices connected to the Raspberry Pi.
#-----------------------------
# For Pet-Mk-VIII (aka "The Dashboard") the output should look like this!
#   I2C Adress '40' = Current/Voltage sensor INA219    <- https://github.com/Pet-Series/pet_ros2_currentsensor_ina219_pkg
#   I2C Adress '48' = Analog/Digital Converter ADS1115 <- https://github.com/Pet-Series/pet_ros2_joystick_pkg
#   I2C Adress '49' = Analog/Digital Converter ADS1115 <- https://github.com/Pet-Series/Pet-Mk-VIII/blob/main/pet_mk_viii/pet_potentiometer_node.py

# 🐳Pet-Mk.VIII_pet@raspikull8:~$ i2cdetect -y 1
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:                         -- -- -- -- -- -- -- -- 
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 40: 40 -- -- -- -- -- -- -- 48 49 -- -- -- -- -- -- 
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 70: -- -- -- -- -- -- -- --                         
# 🐳Pet-Mk.VIII_pet@raspikull8:~$ 
#-----------------------------
i2cdetect -y 1