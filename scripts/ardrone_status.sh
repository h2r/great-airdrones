#!/usr/bin/env bash

batterystatus=$(timeout 1s rostopic echo /ardrone/navdata | head -n 7 | tail -n 1)
if [[ $batterystatus =~ batteryPercent:* ]]; then
    echo -n "$batterystatus"
else
    echo -n "Ardrone disconnected"
fi
