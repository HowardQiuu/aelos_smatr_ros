#!/bin/bash

if test -f /mnt/leju_data/start_init.sh; then
    echo "start_init File exists try to run and auto reboot"
    cp /mnt/leju_data/start_init.sh /tmp/start_init.sh
    sudo bash /tmp/start_init.sh
fi

source /opt/ros/noetic/setup.bash
source /home/lemon/ros_catkin_ws/install_isolated/setup.bash

source /home/lemon/catkin_ws/devel/setup.sh


cd /home/lemon/catkin_ws/src/aelos_smart_ros/
python blue_led_ficker.py &
python start_connect_wifi.py &

cd /home/lemon/catkin_ws/src/aelos_smart_ros/launch  
roslaunch ar_track.launch



