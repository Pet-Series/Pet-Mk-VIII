#!/usr/bin/env python3
# coding = utf-8
'''
    Show CPU-temp a Raspberry PI
    The vcgencmd tool is used to output information from the VideoCore GPU on the Raspberry Pi. 
    '$ vcgencmd' is a command line tool designed by Broadcom used on the Raspberry Pi.
    Interact directly with the hardware or the boot software of a Raspberry pi.  
    Source: <https://www.raspberrypi.com/documentation/computers/os.html#vcgencmd > 
    Source: <https://github.com/raspberrypi/utils/tree/master/vcgencmd > 

    Requires:
        $ sudo apt install -y libraspberrypi-bin

    Other command examples:
        $ vcgencmd measure_temp
        $ vcgencmd get_throttled     => "vcgencmd get_throttled for humans" - https://github.com/alwye/get_throttled
        $ vcgencmd measure_clock arm
        $ vcgencmd get_mem arm
        $ vcgencmd get_mem gpu
        $ vcgencmd get_lcd_info
        $ vcgencmd measure_volts core
        $ vcgencmd measure_volts
        $ vcgencmd measure_clock arm
'''
import os 

#Get CPU temperature using 'vcgencmd measure_temp'                                      
def measure_temp():
    temp = os.popen('vcgencmd measure_temp').readline()
    return(temp.replace("temp=","").replace("'C\n",""))

print( ">> CPU Temp: " + measure_temp())
