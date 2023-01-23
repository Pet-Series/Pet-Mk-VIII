#!/usr/bin/env python3'
# coding = utf-8
########################################################################################
##
## Maintainer: stefan.kull@gmail.com
## 
## Input: Switches/Button that is wired to each GPIO-pin
## Output: ROS2 node that publish topic 
##    joystick_button = HIGH/LOW  (default LOW)
##    main_switch     = HIGH/LOW  (HIGH when activeated)
##
## Prerequisite:
## $ sudo apt-get install python3-rpi.gpio
##
## Hardware: Switch/Button wired to an GPIO-pin
## Host: Raspberry Pi 4(Ubuntu) via I2C
##
## Launch sequence:
## 1) $ ros2 run pet_mk_viii pet_buttons_node.py 
## 2) $ ros2 topic echo /main_switch
##    $ ros2 topic echo /joystick_button
##

# Import the ROS2-stuff
import rclpy 
from rclpy.node import Node
from std_msgs.msg  import Bool

# Import the Ubuntu/Linux-hardware stuff 
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)    # Use GPIO-number for selcting input put.
GPIO.setwarnings(False)   # Get rid of 'RuntimeWarning: This channel is already in use, continuing anyway'

# Import the common Ubuntu/Linux stuff 
import sys
from time import sleep

#Set Button pin(GPIO no.) and ROS2-topic-name
JOYSTICK_BUTTON_PIN = 12
JOYSTICK_BUTTON_ROS_TOPIC = 'joystick_button'

MAIN_SWITCH_PIN = 22
MAIN_SWITCH_ROS_TOPIC = 'main_switch'

class pet_button(Node):
    '''
    A simple ROS2 Push-Button class
    '''
    def __init__(self, pin, topicname, pud_up_down):
        super().__init__("button_node_"+topicname+"_")
        self.get_logger().info("Push/Switch button_node has started")
        self.get_logger().info("- GPIO Pin = " + str(pin))
        self.get_logger().info("- ROS2-topic = " + topicname)
        GPIO.setup(pin, GPIO.IN, pull_up_down=pud_up_down)
        GPIO.add_event_detect(pin, GPIO.BOTH,  callback=self.button_callback, bouncetime=300)

        # Create topic publisher
        self.msg_button = Bool()
        self.msg_button.data = False
        self.pub_button = self.create_publisher(Bool, topicname ,10)

        # Publish intiate/start values
        if GPIO.input(pin): 
            self.msg_button.data = True
        else:
            self.msg_button.data = False       
        self.pub_button.publish(self.msg_button)
        
    def button_callback(self, channel):
        sleep(0.1)

        # Detect if the button was pressed or released
        if GPIO.input(channel): 
            #print("Rising edge detected on " + str(channel) )
            self.msg_button.data = True
        else:
            #print("Falling edge detected on " + str(channel) )
            self.msg_button.data = False
        
        self.pub_button.publish(self.msg_button)
             
def main(args=None):
    rclpy.init(args=args)
    main_switch_node     = pet_button(MAIN_SWITCH_PIN,     MAIN_SWITCH_ROS_TOPIC,     GPIO.PUD_DOWN)
    joystick_button_node = pet_button(JOYSTICK_BUTTON_PIN, JOYSTICK_BUTTON_ROS_TOPIC, GPIO.PUD_UP)

    try:
        rclpy.spin(main_switch_node)  # TODO: Only 'spin' one of two nodes/buttons??? But is seems to work!

    except KeyboardInterrupt:
        print("**** * 💀 Ctrl-C detected...")
    
    finally:
        print("**** 🪦 buttons_node ending... ")
        print( str(sys.exc_info()[1]) )  # Need ´import sys´
        # Time to clean up stuff!
        rclpy.shutdown()
        GPIO.cleanup() 

if __name__ == "__main__":
    main()
