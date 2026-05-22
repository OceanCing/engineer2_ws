#!/bin/bash

source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=~/rm_ws:$ROS_PACKAGE_PATH
source ~/rm_ws/devel/setup.bash

if [[ ! -d ~/Documents/sentry/ ]]; then
	mkdir -p ~/Documents/sentry/
fi
cd ~/Documents/sentry/

BAG_DIR=~/Documents/sentry/

START_TIME=$(date "+%Y%m%d_%H%M")

BAG_FILE="${BAG_DIR}${START_TIME}.bag"

TOPICS="/livox/lidar /livox/imu /ground_segmentation/ground_cloud /tf /cmd_vel
/map /move_base_flex/global_costmap/costmap /move_base_flex/local_costmap/costmap /move_base_flex/GlobalPlanner/plan
/processor/marker /rm_ecat_hw/dbus /rm_ecat_hw/joint_state /rm_ecat_hw/rm_readings
/rm_referee/game_robot_hp /rm_referee/game_robot_status /rm_referee/game_status /rm_referee/rfid_status_data /rm_referee/power_heat_data
/controllers/shooter_controller/trigger/state
/controllers/gimbal_controller/error /sentry_state /sentry_cmd"
DURATION=1200

rosbag record -O $BAG_FILE --duration=$DURATION $TOPICS &
echo "start recording rosbag,file name:$BAG_FILE"

RECORD_PID=$!

trap "echo 'system is shutting down,stop recording rosbag'; kill $RECORD_PID; exit" SIGTERM SIGINT

wait $RECORD_PID