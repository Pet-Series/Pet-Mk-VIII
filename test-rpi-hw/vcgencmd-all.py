#!/usr/bin/env python3
# coding = utf-8
'''
    Show all the accessible information from Broadcom VideoCore GPU on a Raspberry Pi.
    The vcgencmd tool is used to output information from the Broadcom VideoCore GPU on the Raspberry Pi. 
    '$ vcgencmd' is a command line tool designed by Broadcom used on the Raspberry Pi.
    Interact directly with the hardware or the boot software of a Raspberry pi.  
    Source: <https://www.raspberrypi.com/documentation/computers/os.html#vcgencmd> 
    Source: <https://github.com/raspberrypi/utils/tree/master/vcgencmd> 
    Source: <https://github.com/chewett/RaspberryPiVcgencmd>

    Requires:
        $ python3 -m pip install RaspberryPiVcgencmd  
        $ sudo apt install -y libraspberrypi-bin  <- ???
'''
from RaspberryPiVcgencmd import Vcgencmd

sonsor = Vcgencmd()

print(sonsor.get_cpu_temp(), "'C")
print(sonsor.get_cpu_temp(fahrenheit=True), "'F")

print("Ram split", sonsor.get_ram_split())

voltages_to_test = ["core", "sdram_c", "sdram_i", "sdram_p"]
for voltage in voltages_to_test:
    print(voltage, "voltage", sonsor.measure_volts(voltage))

clocks_to_measure = ["arm", "core", "h264", "isp", "v3d", "uart", "pwm", "emmc", "pixel", "vec", "hdmi", "dpi"]
for clock in clocks_to_measure:
    print(clock, "speed", sonsor.measure_clock(clock))

codecs_to_test = ["H264", "MPG2", "WVC1", "MPG4", "MJPG", "WMV9"]
for codec in codecs_to_test:
    print(codec, "available", sonsor.is_codec_available(codec))

print("Version: ", sonsor.get_version())