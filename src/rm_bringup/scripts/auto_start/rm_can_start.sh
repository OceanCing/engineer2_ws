#!/bin/bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can2 up type can bitrate 1000000
sudo ip link set can3 up type can bitrate 1000000
source /opt/ros/noetic/setup.bash
source /home/dynamicx/rm_ws/devel/setup.bash
source /home/dynamicx/mpc_ws/devel/setup.bash
source /home/dynamicx/environment.sh
source /home/dynamicx/intel/openvino_2022/setupvars.sh
if [[ $HAS_SWITCH == has ]]; then
  export ROS_IP=192.168.100.2
else
  export ROS_IP=127.0.0.1
fi
mon launch --disable-ui --log=/tmp/hw.log rm_bringup $LAUNCH.launch
