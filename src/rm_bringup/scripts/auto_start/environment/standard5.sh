#!/bin/bash
# Modifier following environment variable when deploy robot.
# basic
export ENEMY_COLOR=red
ip addr show | grep -q -w '192.168.100.2/24' && export HAS_SWITCH=has || export HAS_SWITCH=no
export ROBOT_TYPE=standard5
export IMU_TRIGGER=false
export HW_NAME=rm_ecat_hw
export LAUNCH=ecat_start
# camera
export CAMERA_TYPE=hk_camera
export CAMERA_CLASS=HKCameraNodelet
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH