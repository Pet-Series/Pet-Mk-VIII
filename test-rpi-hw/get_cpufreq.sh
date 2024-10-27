#!/bin/bash
# Show CPU-frequence on a Raspberry PI
# 🐳Pet-Mk.VIII_pet@raspikull8:~$ ./get_cpufreq.sh
echo ">> CPU Freq:" $(($(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)/1000))MHz