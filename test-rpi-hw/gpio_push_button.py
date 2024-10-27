#! /usr/bin/env python3
# coding = utf-8
'''
# Detecting buttons/Switches (input) for Raspberry Pi, using the "old" RPi.GPIO" library
# Some compatible issues with newer with kernel 5.11...
#
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
import RPi.GPIO as GPIO
from time import sleep
import sys

#Set warnings off (optional)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

#Set Button and LED pins
JOYSTICK_BUTTON = 12   # 'GPIO.BCM' GPIO12 = 'GPIO.BOARD' pin '32'
MAIN_SWITCH = 22       # 'GPIO.BCM' GPIO22 = 'GPIO.BOARD' pin '15'
LED = 6                # 'GPIO.BCM' GPIO6  = 'GPIO.BOARD' pin '31'

class button():
    '''
    A simple Push-Button class
    '''
    def __init__(self, pin, pud_up_down):
        print("'def __init__(self," + str(pin)+ "): '")
        GPIO.setup(pin, GPIO.IN, pull_up_down=pud_up_down)
        GPIO.setup(LED,GPIO.OUT)
        GPIO.add_event_detect(pin, GPIO.BOTH,  callback=self.push_button_callback, bouncetime=300)
#        GPIO.add_event_detect(pin, GPIO.FALLING, callback=self.release_button_callback, bouncetime=300)


    def push_button_callback(self, channel):
        print(channel)
        sleep(0.1)
        if GPIO.input(channel): 
            print("Rising edge detected on " + str(channel) )
            GPIO.output(LED,GPIO.HIGH)  
        else:
            print("Falling edge detected on " + str(channel) )
            GPIO.output(LED,GPIO.LOW)


def main(args=None):
    main_switch     = button(MAIN_SWITCH,     GPIO.PUD_DOWN)
    joystick_button = button(JOYSTICK_BUTTON, GPIO.PUD_UP)
    try:
        while True:
            print(".")
            sleep(5)

    except KeyboardInterrupt:
        print("LedLightNode **** 💀 Ctrl-C detected...")

    finally:
        print("LedLightNode **** 🪦 Ending... ")
        print( str(sys.exc_info()[1]) )  # Need ´import sys´
        # Time to clean up stuff!
        GPIO.cleanup() 

if __name__ == "__main__":
    main()