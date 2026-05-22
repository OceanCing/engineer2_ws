#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/dynamicx/rm_ws/devel/setup.bash
source /home/dynamicx/mpc_ws/devel/setup.bash
source /home/dynamicx/environment.sh
if [[ $HAS_SWITCH == has ]]; then
  export ROS_IP=192.168.100.2
else
  export ROS_IP=127.0.0.1
fi
mon launch --disable-ui --log=/tmp/hw.log rm_bringup $LAUNCH.launch
