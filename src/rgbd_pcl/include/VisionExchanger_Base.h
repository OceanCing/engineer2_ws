//
// Created by hiling on 2026/2/5.
//
#pragma once
#include <iostream>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/TransformStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
// #include <rgbd_pcl/HsvThresholdConfig.h>

#include <boost/bind/bind.hpp>




// FC short for front camera(前置深度相机)
// SC short for side camera(机械臂上相机)
enum class DetectStatus {
    NONE,
    //前置识别同心圆
    FC_FrontBar,
    //前置识别侧灯条
    FC_SideBar,
    //机械臂识别同心圆
    // SC_dount,
    //机械臂识别底部标签+同心圆
    SC_FrontBar
};

