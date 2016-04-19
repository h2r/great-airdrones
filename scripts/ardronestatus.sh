#!/usr/bin/env bash

batterystatus=$(cat ~/Downloads/ardrone_battery)
if [[ $batterystatus =~ batteryPercent:* ]]; then
    echo -n "$batterystatus"
else
    echo -n "Ardrone disconnected"
fi
