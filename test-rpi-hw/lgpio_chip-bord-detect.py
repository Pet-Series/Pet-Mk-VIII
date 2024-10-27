#! /usr/bin/env python3
# coding = utf-8
"""
Raspberry Pi Processor & Type
For more info: https://raspberrypi.stackexchange.com/questions/148477/how-to-determine-the-correct-gpio-chip-for-libgpiod
"""
import sys
# 2024-07-13

def get_revision():
    """
    Returns Raspberry Pi Revision Code
    """
    with open("/proc/device-tree/system/linux,revision", "rb") as fp:
        return int.from_bytes(fp.read(4), 'big')

def processor():
    """
    Raspberry Pi SOC  (https://www.raspberrypi.com/documentation/computers/processors.html)
    returns
        0: BCM2835  RPi1 + Zero
        1: BCM2836  RPi2
        2: BCM2837  RPi3
        3: BCM2711  RPi4
        4: BCM2712  RPi5!
    """
    return int((get_revision()>>12)&7)

def type():
    """
    Raspberry Pi Type
    returns
    0: A
    1: B
    2: A+
    3: B+
    4: 2B
    6: CM1
    8: 3B
    9: Zero
    a: CM3
    c: Zero W
    d: 3B+
    e: 3A+
    10: CM3+
    11: 4B
    12: Zero 2 W
    13: 400
    14: CM4
    15: CM4S
    17: 5
    """
    return int((get_revision()>>4)&0xff)

def main():
    print("RPi detector")
    print(f"RPi Rev. : {get_revision():08x}")
    print(f"RPi SOC  : {processor()}")
    print(f"RPI Model: {type():02x}")
    
if __name__ == '__main__':
    main()