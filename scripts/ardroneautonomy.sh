#!/usr/bin/env bash

if [[ `iwgetid -r` == 'arachna' ]]; then
    if [[ -n `nmap -p 23 $1 | grep open` ]]; then
        if [[ `rostopic list | grep ardrone/front/image_raw` == "" ]]; then
            $(rosrun ardrone_autonomy ardrone_driver -ip $1 &>/dev/null) &
            sleep 2;
        fi

        if [[ `rostopic list | grep ardrone` == "" ]]; then
            echo "Still no ardone autonomy topic, I don't know actually why ;(";
            exit 1;
        fi

        if [[ `rostopic list | grep ardrone/front/image_rect` == "" ]]; then
            $(ROS_NAMESPACE=ardrone/front rosrun image_proc image_proc &>/dev/null) &
            sleep 2;
        fi

        if [[ `rostopic list | grep ardrone/front/image_rect` == "" ]]; then
            echo "Still no image processing topic, I don't know actually why ;("
            exit 1;
        fi
    else
        echo "I can't ping on $1:23"
    fi
else
    echo "You aren't connected to arachna network"
fi

exit 0;
