#!/usr/bin/env python3
import os 

#Get CPU temperature using 'vcgencmd measure_temp'                                      
def measure_temp():
    temp = os.popen('vcgencmd measure_temp').readline()
    return(temp.replace("temp=","").replace("'C\n",""))

print( ">> CPU Temp: " + measure_temp())
