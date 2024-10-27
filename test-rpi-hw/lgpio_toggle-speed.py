#! /usr/bin/env python3
# coding = utf-8
"""
    This program toggles a GPIO pin and measures rate.
    Using lgpio Python Interface

    $ sudo apt-get python3-pip
    $ pip3 install python3-lgpio
    $ pip3 install python3-rpi-lgpio 

"""
# 2024-04-02
# 2024-04-24  commas in number
# 2024-07-13  modified for Pi5/BCBM2712

import lgpio
import time
gpiochip = 0
SigOUT = 12
LOOPS = 20000

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

print("RPi toggle GPIO-speed")
print(f"RPi Rev. : {get_revision():08x}")
print(f"RPi SOC  : {processor()}")

if(processor()==4):
    gpiochip = 4

h = lgpio.gpiochip_open(gpiochip)
if h < 0:
   print('open error')

lgpio.gpio_claim_output(h, SigOUT)

t0 = time.time()

for i in range(LOOPS):
    lgpio.gpio_write(h, SigOUT, 1)
    lgpio.gpio_write(h, SigOUT, 0)

t1 = time.time()
nloops = (1.0 * LOOPS) / (t1 - t0)

print(f"lgpio Python\t{nloops:10,.0f} toggles per second")
lgpio.gpio_free(h, SigOUT)