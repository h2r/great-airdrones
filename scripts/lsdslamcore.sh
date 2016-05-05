#!/usr/bin/env bash

if [[ `iwgetid -r` == 'arachna' ]]; then
    if [[ `rostopic list | grep /ardrone/front/image_rect` == "" ]]; then
        echo "No image stream available;("
        exit 1;
    fi

    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    rosrun lsd_slam_core live_slam /image:=/ardrone/front/image_rect _calib:=$DIR/calibration.cfg &>/dev/null &
else
    echo "You aren't connected to arachna network"
fi

exit 0;
