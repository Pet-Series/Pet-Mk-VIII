import RPi.GPIO as GPIO
from time import sleep
import sys
# Output from $ pinout
#    3V3  (1) (2)  5V    
#  GPIO2  (3) (4)  5V    
#  GPIO3  (5) (6)  GND   
#  GPIO4  (7) (8)  GPIO14
#    GND  (9) (10) GPIO15
# GPIO17 (11) (12) GPIO18
# GPIO27 (13) (14) GND   
# GPIO22 (15) (16) GPIO23
#    3V3 (17) (18) GPIO24
# GPIO10 (19) (20) GND   
#  GPIO9 (21) (22) GPIO25
# GPIO11 (23) (24) GPIO8 
#    GND (25) (26) GPIO7 
#  GPIO0 (27) (28) GPIO1 
#  GPIO5 (29) (30) GND   
#  GPIO6 (31) (32) GPIO12
# GPIO13 (33) (34) GND   
# GPIO19 (35) (36) GPIO16
# GPIO26 (37) (38) GPIO20
#    GND (39) (40) GPIO21

LED = 35 # Same thing as GPIO19

GPIO.setwarnings(False)                     # ignore warningas
 
# GPIO.setmode(GPIO.BCM)                     # GPIO pin numbering
GPIO.setmode(GPIO.BOARD)                    # physical pin numbering
 # set pin 8 as output, initial to low (off)

class LedLightNode():
    """
    A simple LED-example class
    """
    def __init__(self, name):
        print("'def __init__(self): - Run oence'")
        GPIO.setup(LED, GPIO.OUT, initial=GPIO.LOW)

    def blink(self):
        try:
            while True:
                GPIO.output(LED, GPIO.HIGH)  # HIGH / On
                sleep(1)                     # sleep 1 second
                GPIO.output(LED, GPIO.LOW)   # LOW / Off
                sleep(1) 

        except KeyboardInterrupt:
            print("LedLightNode **** 💀 Ctrl-C detected...")

        finally:
            print("LedLightNode **** 🪦 Ending... ")
            print( str(sys.exc_info()[1]) ) # Need ´import sys´
            
            # Time to clean up stuff!
            GPIO.cleanup() 

def main(args=None):
    LED1 = LedLightNode('LED1')
    LED1.blink()

if __name__ == "__main__":
    main()
