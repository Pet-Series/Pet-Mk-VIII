import RPi.GPIO as GPIO
from time import sleep
import sys

#Set warnings off (optional)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

#Set Button and LED pins
JOYSTICK_BUTTON = 12
MAIN_SWITCH = 22
LED = 6

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