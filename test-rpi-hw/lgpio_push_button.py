#!/usr/bin/env python3
# coding = utf-8
'''
    Blink an LED with the LGPIO library
    Uses lgpio library, compatible with kernel 5.11->6.x

    $ sudo apt-get python3-pip
    $ pip3 install python3-lgpio
    $ pip3 install python3-rpi-lgpio 

    Example Raspberry Pi pin-mapping for Pet-Mk.VIII (aka. 'The dashboard')
    $ pinout
                   3V3  (1) (2)  5V    
                 GPIO2  (3) (4)  5V    
                 GPIO3  (5) (6)  GND   
                 GPIO4  (7) (8)  GPIO14
                   GND  (9) (10) GPIO15
                GPIO17 (11) (12) GPIO18
                GPIO27 (13) (14) GND   
 "MAIN_SWITCH"  GPIO22 (15) (16) GPIO23
                   3V3 (17) (18) GPIO24
                GPIO10 (19) (20) GND   
                 GPIO9 (21) (22) GPIO25
                GPIO11 (23) (24) GPIO8 
                   GND (25) (26) GPIO7 
                 GPIO0 (27) (28) GPIO1 
        "LED4"   GPIO5 (29) (30) GND   
        "LED3"   GPIO6 (31) (32) GPIO12   "JOYSTICK_BUTTON"
        "LED2"  GPIO13 (33) (34) GND   
        "LED1"  GPIO19 (35) (36) GPIO16
        "LED0"  GPIO26 (37) (38) GPIO20
                GND (39) (40) GPIO21
'''

import lgpio
from time import sleep
import sys


#Set Button and LED GPIO##-pins
JOYSTICK_BUTTON = 12
MAIN_SWITCH = 22    # GPIO05 (Pin 29) <- ROS2 Topic "/led4"

LED4 =  5           # GPIO05 (Pin 29) <- ROS2 Topic "/led4"
LED3 =  6           # GPIO06 (Pin 31) <- ROS2 Topic "/led3"
LED2 = 13           # GPIO13 (Pin 33) <- ROS2 Topic "/led2"
LED1 = 19           # GPIO19 (Pin 35) <- ROS2 Topic "/led1"
LED0 = 26           # GPIO26 (Pin 37) <- ROS2 Topic "/led0"

class button():
    '''
    A Push/Switch-Button class, for SBC as Raspberry Pi, using lgpio library

    '''
    def __init__(self, pin_button, lFlags, pin_led):
        print("****'def __init__(self, pin_button=" + str(pin_button)+ "): - Run once'")
        self.pin_button = pin_button
        self.pin_led = pin_led
        self.lFlags = lFlags

        # open the gpio chip (via '/dev/gpiochip0')
        self.h = lgpio.gpiochip_open(0)

        # Set the GPIO-'pin_button' as input
        #   eFlags = RISING_EDGE, FALLING_EDGE, BOTH_EDGES
        #   lFlags = SET_ACTIVE_LOW, SET_OPEN_DRAIN, SET_OPEN_SOURCE, SET_PULL_UP, SET_PULL_DOWN, SET_PULL_NONE
        lgpio.gpio_claim_alert(self.h, self.pin_button, lgpio.BOTH_EDGES, self.lFlags)
        lgpio.gpio_set_debounce_micros(self.h, self.pin_button, 300)

        # Read the initial state, if on/off during script __init__.
        self.button_state = lgpio.gpio_read(self.h, self.pin_button)

        # Set the GPIO-'pin' as output
        lgpio.gpio_claim_output(self.h, self.pin_led)
        lgpio.gpio_write(self.h, self.pin_led, self.button_state)

        cb_instance = lgpio.callback(self.h , self.pin_button, lgpio.BOTH_EDGES, push_button_callback)


    def __exit__(self, exc_type, exc_value, exc_traceback):
        print("\nInside __exit__")
        print("\nExecution type:", exc_type)
        print("\nExecution value:", exc_value)
        print("\nTraceback:", traceback)

    def __enter__(self):
        print("__enter__")

    def __del__(self):
        print("__del__")
        # Time to clean up stuff!
        lgpio.gpio_write(self.h, self.pin_led, 0)
        lgpio.gpiochip_close(self.h)

def push_button_callback(chip, pin_button, level, timestamp):
    '''
    Callback when buttons is pressed/changes

    '''
    #print(chip, pin_button, level, timestamp)

    # level == 0: change to low (a falling edge)
    if pin_button == JOYSTICK_BUTTON and level == 0:
        print("Button pressed " + str(pin_button) )

    # level=1: change to high (a rising edge)
    if pin_button == JOYSTICK_BUTTON  and level == 1:
        print("Button released " + str(pin_button) )

    # level == 0: change to low (a falling edge)
    if pin_button == MAIN_SWITCH and level == 0:
        print("Switch deactivated(off) " + str(pin_button) )

    # level=1: change to high (a rising edge)
    if pin_button == MAIN_SWITCH and level == 1:
        print("Switch activated(on) " + str(pin_button) )

    if pin_button == MAIN_SWITCH and level == 2:
        print("Watchdog " + str(pin_button) )

def main(args=None):
    joystick_button = button(JOYSTICK_BUTTON, lgpio.SET_PULL_UP,   LED4)
    main_switch     = button(MAIN_SWITCH,     lgpio.SET_PULL_DOWN, LED0)

    try:
        while True:
            print(".")
            sleep(2.5)

    except KeyboardInterrupt:
        print("LedLightNode **** 💀 Ctrl-C detected...")

    finally:
        print("LedLightNode **** 🪦 Ending... ")
        print( str(sys.exc_info()[1]) )  # Need ´import sys´


if __name__ == "__main__":
    main()