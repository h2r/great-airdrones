#!/usr/bin/env bash

if [[ `iwgetid -r` == 'arachna' ]]; then
    if [[ -n `nmap -p 3883 $1 | grep open` ]]; then
        if [[ `rostopic list | grep vrpn_client_node` != "" ]]; then
            roslaunch vrpn_client_ros sample.launch server:=$1 &>/dev/null &
        fi

        if [[ `rostopic list | grep vrpn_client_node` == "" ]]; then
            echo "Still no topic, I don't know actually why ;("
        fi
    else
        echo "I can't ping on $1:3883"
    fi
else
    echo "You aren't connected to arachna network"
fi
