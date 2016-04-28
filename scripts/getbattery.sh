#!/bin/bash
while true; do rostopic echo /ardrone/navdata 2>&1 | head -n 7 2>/dev/null | tail -n 1 2>/dev/null 1>/tmp/ardrone_battery; sleep 10; done;
