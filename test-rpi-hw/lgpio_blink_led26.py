#!/usr/bin/env python3
# coding = utf-8
''' 
    Raspberry Pi - Blink an LED using the LGPIO library
    Uses lgpio library, compatible with Linux kernel 5.11->6.x

    $ sudo apt-get python3-pip
    $ pip3 install python3-lgpio
    $ pip3 install python3-rpi-lgpio     

    Example pin-mapping for Pet-Mk.VIII (aka. 'The dashboard')
    $ pinout
                   3V3  (1) (2)  5V    
                 GPIO2  (3) (4)  5V    
                 GPIO3  (5) (6)  GND   
                 GPIO4  (7) (8)  GPIO14
                   GND  (9) (10) GPIO15
                GPIO17 (11) (12) GPIO18
                GPIO27 (13) (14) GND   
                GPIO22 (15) (16) GPIO23
                   3V3 (17) (18) GPIO24
                GPIO10 (19) (20) GND   
                 GPIO9 (21) (22) GPIO25
                GPIO11 (23) (24) GPIO8 
                   GND (25) (26) GPIO7 
                 GPIO0 (27) (28) GPIO1 
        "LED4"   GPIO5 (29) (30) GND   
        "LED3"   GPIO6 (31) (32) GPIO12
        "LED2"  GPIO13 (33) (34) GND   
        "LED1"  GPIO19 (35) (36) GPIO16
        "LED0"  GPIO26 (37) (38) GPIO20
                   GND (39) (40) GPIO21
'''
import time
import lgpio
import sys

LED4 =  5 # GPIO05 (Board pin 29) <- ROS2 Topic "/led4"
LED3 =  6 # GPIO06 (Board pin 31) <- ROS2 Topic "/led3"
LED2 = 13 # GPIO13 (Board pin 33) <- ROS2 Topic "/led2"
LED1 = 19 # GPIO19 (Board pin 35) <- ROS2 Topic "/led1"
LED0 = 26 # GPIO26 (Board pin 37) <- ROS2 Topic "/led0"

class LedLightNode():
    """
    A simple LED-example class
    """
    def __init__(self, pin):
        print("'def __init__(self): - Run once'")
        self.pin = pin
        # open the gpio chip and set the 'pin' as output
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, self.pin)

    def blink(self):
        try:
            print("Spinning [Pin" + str(self.pin) + "]")
            while True:
                print(".")
                # Turn the GPIO pin on
                lgpio.gpio_write(self.h, self.pin, 1)
                time.sleep(1)

                # Turn the GPIO pin off
                lgpio.gpio_write(self.h, self.pin, 0)
                time.sleep(1)

        except KeyboardInterrupt:
            print("LedLightNode **** 💀 Ctrl-C detected...")

        finally:
            print("LedLightNode **** 🪦 Ending... ")
            print( str(sys.exc_info()[1]) ) # Need ´import sys´
            
            # Time to clean up stuff!
            lgpio.gpio_write(self.h, self.pin, 0)
            lgpio.gpiochip_close(self.h)

def main(args=None):
    LED = LedLightNode(LED0)
    LED.blink()

if __name__ == "__main__":
    main()