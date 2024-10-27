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
## Input: Analog potentiometer chn. 1 + 2 + 3 (+4 not used at the moment on the ADS1115 chip)
## Output: ROS2 node that publish topic potentiometer_p... with msg.type Int32
##
## Behaviour:
##   1) Once: Read/Set all the parameters with the following default values for node parameters
##      1.1) List all parameters for a running node
##          $ ros2 param dump /PotentiometerPublisher_node
##              /PotentiometerPublisher_node:
##                  ros__parameters:
##                      adc_i2c_address: '0x49'
##                      cycle_timer: 0.1
##                      cycles_publish: 50
##                      granularity: 5
##                      ros_topic_p0: potentiometer_p0  
##                      ros_topic_p1: potentiometer_p1
##                      ros_topic_p2: potentiometer_p2
##                      ros_topic_p3: potentiometer_p3
##                      scale_p0: 1
##                      scale_p1: 1
##                      scale_p2: 1
##                      scale_p3: 1
##                      use_sim_time: false
##      1.2) To override one of the parameters (Example Scale from 0..2000 for potentiometer 2)
##          $ ros2 run pet_mk_viii pet_potentiometer_node --ros-args --param scale_p1:=2
##
##      1.3) To override ALL parameters from a .yaml file  (file syntax see: ros2 param dump /PotentiometerPublisher_node )
##          $ ros2 run pet_mk_viii pet_potentiometer_node --ros-args --params-file $HOME/ws_ros2/src/Pet-Mk-VIII/pet_mk_viii/pet_potentiometer_node.py
##
##   2) Repeatedly at 10Hz (See parameter 'cycle_timer: 0.1') 
##      2.1) Read analog potentiometer via ADC as raw-output
##      2.2) Transform the raw-ADC-output to a value between -100 and +100 and number of distinct values set by the granularity
##      2.3) Repeatedly at 10Hz: Decide to publish ROS2 topic name or not (See parameter 'ros_topic_p0..3') depending on the following two rules
##          2.3.1) Value change? If Yes => Then Publish all ROS2 topics (See variable 'doPublish = True')
##          2.3.2) No change/publishing for 5 sec? If Yes => Then publish all ROS2 topics (See parameter 'cycles_publish: 50') 
##
## Prerequisite / Dependencies: 
##   $ sudo apt install i2c-tools
##   $ sudo i2cdetect -y 1
##   $ sudo apt install python3-pip
##   $ sudo pip3 install smbus2
##   $ sudo pip3 install adafruit-ads1x15
##   $ sudo chmod a+rw /dev/i2c-1
##
##
## Prerequisite / Hardware
##  1) KY-053 ADC "Analog Digital Converter" (ADS1115, 16-bit) via default I2C adr.=0x49
##  2) 3x Analog potentiometer that has about 10K resistors
##  3) Hardware/SBC: Raspberry Pi 3..5 (Ubuntu or Raspian OS) using I2C
##
## Launch sequence:
##   0) $ source ./install/setup.bash
##   1) $ ros2 run pet_mk_viii pet_potentiometer_node.py
##          [INFO] [1726052923.990975972] [PotentiometerPublisher_node]: PotentiometerPublisher_node has started
##          [INFO] [1726052923.993765948] [PotentiometerPublisher_node]: - A/D: 0x49, P0-chn=potentiometer_p0, P1-chn=potentiometer_p1, P2-chn=potentiometer_p2, P3-chn=potentiometer_p3
##          [INFO] [1726052923.996299631] [PotentiometerPublisher_node]: - A/D sampling: 10.0Hz
##          [INFO] [1726052924.102641013] [PotentiometerPublisher_node]: -----Publish P0!
##          [INFO] [1726052924.110070988] [PotentiometerPublisher_node]: -----Publish P1!
##          [INFO] [1726052924.113278831] [PotentiometerPublisher_node]: -----Publish P2!
##          [INFO] [1726052924.115883327] [PotentiometerPublisher_node]: -----Publish P3!
##   2) $ ros2 topic list
##          /parameter_events
##          /potentiometer_p0
##          /potentiometer_p1
##          /potentiometer_p2
##          /potentiometer_p3
##   2) $ ros2 topic echo /potentiometer_p0
##      $ ros2 topic echo /potentiometer_p2
##      $ ros2 topic echo /potentiometer_p3
##          ---
##          data: 0
##          ---
##          data: 500
##          ---
##          data: 1000
##          ---
##

# TODO: Make the scale factor of type 'DOUBLE' (Today it is of type 'INTEGER')
# TODO: Make the SCALE-MIN & SCALE_MAX values as param. (Today hardcoded as 0 and 100)
# TODO: Get rid of time.sleep() with something more real time/concurrent and ROS2 friendly way of wait...

# Import the ROS2-stuff
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Int32

# Import the Ubuntu/Linux-hardware stuff 
import Adafruit_ADS1x15

# Import the common Ubuntu/Linux stuff 
import sys
import time

class PotentiometerPublisher(Node): 
    '''
    Analog potentiometer class
    Read analog input -> Publish on ROS2 topic
    '''
    
    # Keep track of last joystick values. Used due to reducing communication of equal values.
    last_value_p0 = 0
    last_value_p1 = 0
    last_value_p2 = 0
    last_value_p3 = 0

    def __init__(self):
        super().__init__("PotentiometerPublisher_node")
        
        # Set default topic-name for publishing. Accessed via ROS Parameters...
        self.declare_parameter( 'ros_topic_p0', 'potentiometer_p0', ParameterDescriptor(description='ROS2 topic name. Publish position of potentiometer (Not yet used on Pet-Mk.VIII) [default "potentiometer_p0"]') )
        self.ROS_TOPIC_P0 = self.get_parameter('ros_topic_p0').get_parameter_value().string_value

        self.declare_parameter( 'ros_topic_p1', 'potentiometer_p1', ParameterDescriptor(description='ROS2 topic name. Publish position of potentiometer [default "potentiometer_p1"]') )
        self.ROS_TOPIC_P1 = self.get_parameter('ros_topic_p1').get_parameter_value().string_value

        self.declare_parameter( 'ros_topic_p2', 'potentiometer_p2', ParameterDescriptor(description='ROS2 topic name. Publish position of potentiometer [default "potentiometer_p2"]') )
        self.ROS_TOPIC_P2 = self.get_parameter('ros_topic_p2').get_parameter_value().string_value

        self.declare_parameter( 'ros_topic_p3', 'potentiometer_p3', ParameterDescriptor(description='ROS2 topic name. Publish position of potentiometer [default "potentiometer_p3"]') )
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

        # Granularity is the values step size between. Accessed via ROS Parameters...
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
        # Check we can open the analog/digital converter, ads1115, via I2C-interface.
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
        # Read values from potentiometers P0, P1, P2 and P3
        # With gain=1 the "raw"-value(1...26400) and middle 13200
        # With gain=2/3 the "raw"-value(1...17600) and middle 8900
        # My ads1115 needed a little sleep between measurements to settle on a good value
        p0_raw= self.adc.read_adc(0, gain=1, data_rate=128) # ADS1115 channel 0. 
        time.sleep(0.01)

        p1_raw= self.adc.read_adc(1, gain=1, data_rate=128) # ADS1115 channel 1
        time.sleep(0.01)

        p2_raw= self.adc.read_adc(2, gain=1, data_rate=128) # ADS1115 channel 2
        time.sleep(0.01) 

        p3_raw= self.adc.read_adc(3, gain=1, data_rate=128) # ADS1115 channel 3
        time.sleep(0.01) 
 
        # self.get_logger().info("Raw: P0=" + str(p0_raw) + " P1=" + str(p1_raw) + " P2=" + str(p2_raw))

        # Convert to a value between -100 and +100 and number of distinct values set by the granularity
        value_p0 = ( round((round(p0_raw/26.410) * self.SCALE_P0 ) / self.GRANULARITY)) * self.GRANULARITY
        value_p1 = ( round((round(p1_raw/26.410) * self.SCALE_P1 ) / self.GRANULARITY)) * self.GRANULARITY
        value_p2 = ( round((round(p2_raw/26.410) * self.SCALE_P2 ) / self.GRANULARITY)) * self.GRANULARITY
        value_p3 = ( round((round(p3_raw/26.410) * self.SCALE_P3 ) / self.GRANULARITY)) * self.GRANULARITY
        
        # Only output a message when the joystick values change,
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
        print("**** 🪦 potentiometer_node ending... " + str(sys.exc_info()[1]) )
        # Time to clean up stuff!
        rclpy.shutdown()

if __name__ == "__main__":
    main()
