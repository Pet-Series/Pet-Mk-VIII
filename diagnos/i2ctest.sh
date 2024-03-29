#!/bin/bash
# Test script for i2c access. 
# For Pet-Mk-VIII (aka "The Dashboard") it should look like this!
#-----------------------------
# 🐳Pet-Mk.VIII_pet@raspikull8:~$ i2cdetect -y 1
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:                         -- -- -- -- -- -- -- -- 
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 40: -- -- -- -- -- -- -- -- 48 49 -- -- -- -- -- -- 
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 70: -- -- -- -- -- -- -- --                         
# 🐳Pet-Mk.VIII_pet@raspikull8:~$ 
#-----------------------------
i2cdetect -y 1