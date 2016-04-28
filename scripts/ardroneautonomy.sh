#!/usr/bin/env bash

if [[ `iwgetid -r` == 'arachna' ]]; then
    if [[ -n `nmap -p 5554-5556,5559 $1 | grep open` ]]; then
        rosrun ardrone_autonomy ardrone_driver -ip $1 &>/dev/null &
    else
        echo "I can't ping on $1:5555"
    fi
else
    echo "You aren't connected to arachna network"
fi

