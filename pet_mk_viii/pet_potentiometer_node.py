#!/usr/bin/env python3
# coding = utf-8
########################################################################################
## (c) https://github.com/Pet-Series
##     https://github.com/Pet-Series/Pet-Mk-VIII
##
## Maintainer: stefan.kull@gmail.com
## The MIT License (MIT)
##
## Target platform: Pet-Mk.VIII (aka. the "Dashboard")
## 
## Input: Analog potentiometer chn. 1 + 2 + 3 (+4 not used at the moment)
## Output: ROS2 node that publish topic potentiometer_p... with msg.type Int32
##
## Behaviour:
##   1) Once: Read/Set all the parameters
##   2) Repeatedly: Read analog potentiometer via ADC as raw-output
##   3) Repeatedly: Transform the raw-ADC-output to a value bettween -100 and +100 and number of distict values set by the granularity
##   4) Repeatedly: Publish ros-topic
##
## Prerequisite:
##   $ sudo apt install i2c-tools
##   $ sudo i2cdetect -y 1
##   $ sudo apt install python3-pip
##   $ sudo pip3 install smbus2
##   $ sudo pip3 install adafruit-ads1x15
##   $ sudo chmod a+rw /dev/i2c-1
##
## Hardware: KY-053 ADC "Analog Digital Converter" (ADS1115, 16-bit) via default I2C adr.=0x49
## Hardware: 3x Analog potentiometer that has about 10K resistors
## Hardware/SBC: Raspberry Pi 3-4 (Ubuntu or Raspian OS) via I2C
##
## Launch sequence:
##   1) $ ros2 run pet_mk_viii pet_potentiometer_node.py
##   2) $ ros2 topic echo /potentiometer_p0
##      $ ros2 topic echo /potentiometer_p2
##      $ ros2 topic echo /potentiometer_p3
##

# TODO: Get rid of time.sleep() with something more real time/concurrent and ROS2 friendly way of wait...

# Import the ROS2-stuff
import rclpy  # TODO: IS this line neccesary. Due to the two following lines that importing "Node" and "Parameter"
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Int32

# Import the Ubuntu/Linux-hardware stuff 
from smbus2 import SMBus
import Adafruit_ADS1x15
#from gpiozero import LED

# Import the common Ubuntu/Linux stuff 
import sys
import time
import signal

class PotentiometerPublisher(Node): 
    '''
    Analog potentiometer class
    Read analog input -> Publish on ROS-topic
    '''
    
    # Keep track of last joystick values. Used due to reducing communication of equal values.
    last_value_p0 = 0
    last_value_p1 = 0
    last_value_p2 = 0
    last_value_p3 = 0

    def __init__(self):
        super().__init__("PotentiometerPublisher_node")
        
        # Set default topic-name for publishing. Accessed via ROS Parameters...
        self.declare_parameter( 'ros_topic_p0', 'potentiometer_p0', ParameterDescriptor(description='ROS-topc name. Publish position of potentiometer [default "potentiometer_p0"]') )
        self.ROS_TOPIC_P0 = self.get_parameter('ros_topic_p0').get_parameter_value().string_value

        self.declare_parameter( 'ros_topic_p1', 'potentiometer_p1', ParameterDescriptor(description='ROS-topc name. Publish position of potentiometer [default "potentiometer_p1"]') )
        self.ROS_TOPIC_P1 = self.get_parameter('ros_topic_p1').get_parameter_value().string_value

        self.declare_parameter( 'ros_topic_p2', 'potentiometer_p2', ParameterDescriptor(description='ROS-topc name. Publish position of potentiometer [default "potentiometer_p2"]') )
        self.ROS_TOPIC_P2 = self.get_parameter('ros_topic_p2').get_parameter_value().string_value

        self.declare_parameter( 'ros_topic_p3', 'potentiometer_p3', ParameterDescriptor(description='ROS-topc name. Publish position of potentiometer [default "potentiometer_p3"]') )
        self.ROS_TOPIC_P3 = self.get_parameter('ros_topic_p3').get_parameter_value().string_value

        # Set default ADC-I2C address. Accessed via ROS Parameters...
        self.declare_parameter( 'adc_i2c_address', "0x49", ParameterDescriptor(description='ADC I2C address [default "0x49"]') )
        self.ADC_I2C_ADDRESS = self.get_parameter('adc_i2c_address').get_parameter_value().string_value

        # Scale factor for each potentiometer. Accessed via ROS Parameters...
        self.declare_parameter( 'scale_p0', 1, ParameterDescriptor(description='Scale factor 0->1000 value [default=1]') )
        self.SCALE_P0 = self.get_parameter( 'scale_p0' ).value

        self.declare_parameter( 'scale_p1', 1, ParameterDescriptor(description='Scale factor 0->1000 value [default=1]') )
        self.SCALE_P1 = self.get_parameter( 'scale_p1' ).value
        
        self.declare_parameter( 'scale_p2', 1, ParameterDescriptor(description='Scale factor 0->1000 value [default=1]') )
        self.SCALE_P2 = self.get_parameter( 'scale_p2' ).value 

        self.declare_parameter( 'scale_p3', 1, ParameterDescriptor(description='Scale factor 0->1000 value [default=1]') )
        self.SCALE_P3 = self.get_parameter( 'scale_p3' ).value 

        # Granularity is the values stepsize between. Accessed via ROS Parameters...
        self.declare_parameter( 'granularity', 5, ParameterDescriptor(description='Joystick value step-size [default 5].') )
        self.GRANULARITY = self.get_parameter( 'granularity' ).value

        # Cycle time - How often to read joystick position via A/D converter
        self.declare_parameter( 'cycle_timer', 0.1, ParameterDescriptor(description='Cycle time how often to read joystick position [default 0.1 sec(10Hz)]') )
        self.CYCLE_TIMER = self.get_parameter( 'cycle_timer' ).value

        # Republish every n number of cycles to make sure base got last value
        self.declare_parameter( 'cycles_publish', 50, ParameterDescriptor(description='Number of cycles/timer before publish [default 50]') )
        self.CYCLES_COUNTER = self.get_parameter( 'cycles_publish' ).value
        self.republish_counter = self.CYCLES_COUNTER

        exit = False
        # Check we can open the analog/digitala converter, ads1115, via I2C-interface.
        try:
            self.adc = Adafruit_ADS1x15.ADS1115( busnum=1, address=int(self.ADC_I2C_ADDRESS,0) )  # "0" works for both Dec and Hex strings...

            # Create topic publisher
            self.pub_p0 = self.create_publisher(Int32, self.ROS_TOPIC_P0 ,10)
            self.pub_p1 = self.create_publisher(Int32, self.ROS_TOPIC_P1 ,10)
            self.pub_p2 = self.create_publisher(Int32, self.ROS_TOPIC_P2 ,10)
            self.pub_p3 = self.create_publisher(Int32, self.ROS_TOPIC_P3 ,10)

            # Set cycle time (Hz) how often to read the joystick position.
            self.joy_timer = self.create_timer(self.CYCLE_TIMER , self.process_potentiometers) 
            # TODO: self.rosRunLED.on()

            # Some basic information on the console
            self.get_logger().info("PotentiometerPublisher_node has started")
            self.get_logger().info("- A/D: " + self.ADC_I2C_ADDRESS + ", P0-chn=" + self.ROS_TOPIC_P0 + ", P1-chn=" + self.ROS_TOPIC_P1 + ", P2-chn="  + self.ROS_TOPIC_P2 + ", P3-chn="  + self.ROS_TOPIC_P3)
            self.get_logger().info("- A/D sampling: " + str(100 *self.CYCLE_TIMER) +"Hz")

        except:
            # Note: a permission error can be fixed with a "sudo chmod a+rw /dev/i2c-1"
            self.get_logger().error("During PotentiometerPublisher_node initialization😒"  + str(sys.exc_info()[1]) )
            self.exit = True

       
        #########################################################################################################
        # 1) Initiate & assign start values for the ros-msg.
        # 2) Publish a first time
               
        self.msg_p0 = Int32()
        self.msg_p0.data = 0
        self.msg_p1 = Int32()
        self.msg_p1.data = 0
        self.msg_p2 = Int32()
        self.msg_p2.data = 0  
        self.msg_p3 = Int32()
        self.msg_p3.data = 0         

    def process_potentiometers(self):
        # Read values from potetinometers P0, P1, P2 and P3
        # With gain=1 the "raw"-value(1...26400) and middle 13200
        # With gain=2/3 the "raw"-value(1...17600) and middle 8900
        # My ads1115 needed a little sleep between measuerments to settle on a good value
        p0_raw= self.adc.read_adc(0, gain=1, data_rate=128) # ADS1115 channel 0. 
        time.sleep(0.01)

        p1_raw= self.adc.read_adc(1, gain=1, data_rate=128) # ADS1115 channel 1
        time.sleep(0.01)

        p2_raw= self.adc.read_adc(2, gain=1, data_rate=128) # ADS1115 channel 2
        time.sleep(0.01) 

        p3_raw= self.adc.read_adc(3, gain=1, data_rate=128) # ADS1115 channel 3
        time.sleep(0.01) 
 
        # self.get_logger().info("Raw: P0=" + str(p0_raw) + " P1=" + str(p1_raw) + " P2=" + str(p2_raw))

        # Convert to a value bettween -100 and +100 and number of distict values set by the granularity
        value_p0 = ( round((round(p0_raw/26.410) * self.SCALE_P0 ) / self.GRANULARITY)) * self.GRANULARITY
        value_p1 = ( round((round(p1_raw/26.410) * self.SCALE_P1 ) / self.GRANULARITY)) * self.GRANULARITY
        value_p2 = ( round((round(p2_raw/26.410) * self.SCALE_P2 ) / self.GRANULARITY)) * self.GRANULARITY
        value_p3 = ( round((round(p3_raw/26.410) * self.SCALE_P3 ) / self.GRANULARITY)) * self.GRANULARITY
        
        
        # Only output a twist message when the joystick values change,
        # If nothing published for N loops/iterations - Then send the last value again.
        doPublish = False


        self.republish_counter -= 1


        if (self.last_value_p0 != value_p0):
            # Change in P0 value
            self.get_logger().info("-----Publish P0!")
            self.msg_p0.data = value_p0
            doPublish = True

        if (self.last_value_p1 != value_p1):
            # Change in P1 value
            self.get_logger().info("-----Publish P1!")
            self.msg_p1.data = value_p1
            doPublish = True

        if (self.last_value_p2 != value_p2):
            # Change in P2 value
            self.get_logger().info("-----Publish P2!")
            self.msg_p2.data = value_p2
            doPublish = True

        if (self.last_value_p3 != value_p3):
            # Change in P3 value
            self.get_logger().info("-----Publish P3!")
            self.msg_p3.data = value_p3
            doPublish = True

        if doPublish or self.republish_counter == 0:
            self.get_logger().info("Raw  : P0=" + str(p0_raw)   + " P1=" + str(p1_raw)   + " P2=" + str(p2_raw)   + " P3=" + str(p3_raw))
            self.get_logger().info("Value: P0=" + str(value_p0) + " P1=" + str(value_p1) + " P2=" + str(value_p2) + " P3=" + str(value_p3) )
            self.pub_p0.publish(self.msg_p0)
            self.pub_p1.publish(self.msg_p1)
            self.pub_p2.publish(self.msg_p2)
            self.pub_p3.publish(self.msg_p3)

            self.republish_counter = self.CYCLES_COUNTER  # Restart the counter

        # Save current value, so we can see if the stick have been moved during next lap.
        self.last_value_p0 = value_p0
        self.last_value_p1 = value_p1
        self.last_value_p2 = value_p2
        self.last_value_p3 = value_p3
             
def main(args=None):
    rclpy.init(args=args)
    node = PotentiometerPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("**** * 💀 Ctrl-C detected...")
    
    finally:
        print("**** 🪦 joystick_node ending... " + str(sys.exc_info()[1]) )
        # Time to clean up stuff!
        rclpy.shutdown()

if __name__ == "__main__":
    main()
