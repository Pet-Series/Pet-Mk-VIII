#!/bin/bash
# Show CPU-temp on a Raspberry PI
# 🐳Pet-Mk.VIII_pet@raspikull8:~$ ./get_cputemp.sh
awk '{print $1/1000,"degrees C"}' /sys/class/thermal/thermal_zone0/temp