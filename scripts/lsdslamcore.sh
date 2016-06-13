#!/usr/bin/env bash

    if [[ `rostopic list | grep /ardrone/front/image_rect` == "" ]]; then
        echo "No image stream available;("
        exit 1;
    fi

    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    rosrun lsd_slam_core live_slam /image:=/ardrone/front/image_rect _calib:=$DIR/calibration.cfg &>/dev/null &

exit 0;
