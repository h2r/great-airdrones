#!/usr/bin/env zsh

batterystatus=$(cat /tmp/ardrone_battery)
if [[ $batterystatus =~ batteryPercent:* ]]; then
    echo "$batterystatus"
else
    echo "Ardrone disconnected"
fi
