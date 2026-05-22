#!/bin/bash
# Modifier following environment variable when deploy robot.
# basic
ip addr show | grep -q -w '192.168.100.2/24' && export HAS_SWITCH=has || export HAS_SWITCH=no
export ROBOT_TYPE=spare_sentry
export IMU_TRIGGER=true
export HW_NAME=rm_can_hw
export LAUNCH=sentry
# camera
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export CAMERA_TYPE=galaxy_camera
export CAMERA_CLASS=GalaxyCameraNodelet
export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH