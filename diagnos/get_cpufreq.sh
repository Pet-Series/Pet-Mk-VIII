#!/bin/bash
echo ">> CPU Freq:" $(($(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)/1000))MHz