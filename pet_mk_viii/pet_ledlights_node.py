#!/usr/bin/env python3
# coding = utf-8
########################################################################################
## (c) https://github.com/Pet-Series
##     https://github.com/Pet-Series/pet_ros2_lightbeacon_pkg
##
## Maintainer: stefan.kull@gmail.com
## The MIT License (MIT)
##
## Input:  Subscribe to ROS2-topics
## Output: Power up, swoitch on/off LED x5 (0..4)
##
## Behaviour: 
##   1) Once: Read/Set all the parameters 
##   2) Repeatedly: Subscribe ROS2-topics for "/panel_led0...4" 
##   3) Repeatedly: Switch on/off corresponding LED
##
## Prerequisite   : Linux/Ubuntu vs. Hardware
##   Hardware/SBC   : Raspberry Pi 4(Ubuntu)
##   Hardware/Light : 5x (LED + 1KΩ)
##            +--------+--------+
##        ----+ GPIO05 | LED0   +-----
##            +--------+--------+
##        ----+ GPIO06 | LED1   +-----
##            +--------+--------+
##        ----+ GPIO13 | LED2   +-----
##            +--------+--------+
##    RPi ----+ GPIO19 | LED3   +----- LED lights
##            +--------+--------+
##        ----+ GPIO26 | LED4   +-----
##            +--------+--------+
##        ----+ GND    | Common +-----
##            +--------+--------+
##
## Prerequisite: Linux/Ubuntu vs. Software
##   $ sudo apt install python3-pip
##   $ sudo apt-get install python3-rpi.gpio
##
## Launch sequence (only one):
##   1) $ ros2 run pet-mk-viii pet_ledlights_node 
##        [INFO] [1648473699.905946642] [LedLightNode]: LedLightNode has started 'led1' at GPIO-pin: 6
##   2) $ ros2 topic pub /panel_led_3 std_msgs/msg/Bool "data: True" -1
##      $ ros2 topic pub /panel_led_3 std_msgs/msg/Bool "data: False" -1
##
## Launch sequence (all 5 LED's):
##   1) $ ros2 launch pet_mk_viii panel_led[0..4]_subscribers.launch.py
##        [pet_ledlights_node-4] [INFO]: LedLightNode has started 'panel_led_3' at GPIO-pin: 19
##        [pet_ledlights_node-3] [INFO]: LedLightNode has started 'panel_led_2' at GPIO-pin: 13
##        [pet_ledlights_node-1] [INFO]: LedLightNode has started 'panel_led_0' at GPIO-pin: 5
##        [pet_ledlights_node-2] [INFO]: LedLightNode has started 'panel_led_1' at GPIO-pin: 6
##        [pet_ledlights_node-5] [INFO]: LedLightNode has started 'panel_led_4' at GPIO-pin: 26
##   2) $ ros2 topic pub /panel_led_2 std_msgs/msg/Bool "data: True" -1
##      $ ros2 topic pub /panel_led_2 std_msgs/msg/Bool "data: False" -1
##
## Using ROS2 parameters to setup each individual LED via the .lanuch.pt script above.
##      $ ros2 param dump /LedLightNode --print
##        /LedLightNode:
##          ros__parameters:
##            led_gpio_pin: 26
##            led_topic: panel_led_4

#  Include the ROS2 stuff...
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool

# Importe the Raspberry Pi I/O stuff
import RPi.GPIO as GPIO 

#  Include Python stuff...
import sys
import signal
from time import *

GPIO.setmode(GPIO.BCM) # GPIO pin numbering
LED4 =  5 # GPIO05 (Pin 29)
LED3 =  6 # GPIO06 (Pin 31)
LED2 = 13 # GPIO13 (Pin 33)
LED1 = 19 # GPIO19 (Pin 35)
LED0 = 26 # GPIO26 (Pin 37)

class LedLightNode(Node): 
    '''
    ROS2 LED class for LED's direct wired to individual GPIO-pins.
    '''
    def __init__(self):
        super().__init__("LedLightNode")
        # Set default topic-name for subscription. Accessed via ROS Parameters...
        self.declare_parameter( 'led_topic', 'led1', ParameterDescriptor(description='ROS-topic subscription name. [default "led1"]') )
        self.LED_TOPIC = self.get_parameter('led_topic').get_parameter_value().string_value

        # Set default topic-name for subscription. Accessed via ROS Parameters...
        self.declare_parameter( 'led_gpio_pin', LED1, ParameterDescriptor(description='GPIO-pin connected to LED. [default "LED1"<#26>]') )
        self.LED_GPIO_PIN = self.get_parameter('led_gpio_pin').value

        # Check we can open/contact the GPIO-pins for the Light Beacon
        self.create_subscription(Bool,  self.LED_TOPIC, self.led_light_callback, 10)
        self.get_logger().info("LedLightNode has started '" + self.LED_TOPIC + "' at GPIO-pin: " + str( self.LED_GPIO_PIN ) )
        GPIO.setup( self.LED_GPIO_PIN, GPIO.OUT, initial=GPIO.LOW)

    # Callback when new value set for topic.
    def led_light_callback(self, msg):
        self.get_logger().info("+------led_light_callback(self, msg):<" + self.LED_TOPIC + ", " + str(msg.data) + ", GPIO=" + str(self.LED_GPIO_PIN) + ">-------" )
 
        if msg.data == True:
             GPIO.output(self.LED_GPIO_PIN, GPIO.HIGH)
             print("Tänder")
        
        if msg.data == False:
            GPIO.output(self.LED_GPIO_PIN, GPIO.LOW)      

    

def main(args=None):
    rclpy.init(args=args)
    node = LedLightNode()
    
    #The try statement is used to detect errors in the try block.
    #The except statement catches the exception information and processes it.
    #The finally statement always run. Regardless reson to end the script.
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("LedLightNode **** 💀 Ctrl-C detected...")
    
    finally:
        # 
        print("LedLightNode **** 🪦 Ending... ")
        print( str(sys.exc_info()[1]) )
        
        # Time to clean up stuff!
        # - Destroy the node explicitly
        #   (optional - otherwise it will be done automatically
        #   when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup() 

if __name__ == "__main__":
    main()
