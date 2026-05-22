//
// Created by ch on 2026/3/24.
//
#include "engineer_lifter_controller/engineer_lifter_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Int32.h>

namespace engineer_lifter_controller {
  bool EngineerLifterController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    if (!controller_nh.hasParam("lifter") || !controller_nh.hasParam("wheel")) {
      ROS_ERROR("Lifter or wheel was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
      return false;
    }
    cmd_sub_ =
        controller_nh.subscribe<std_msgs::Int32>("/engineer_lifter_cmd", 1, &EngineerLifterController::lifterCmdCallback, this);
    controller_nh.getParam("lift_position", lift_position_);
    controller_nh.getParam("lift_vel", lift_vel_);
    controller_nh.getParam("up_move_vel", up_move_vel_);
    controller_nh.getParam("down_ready_effort", down_ready_effort_);

    ros::NodeHandle left_lifter_nh(controller_nh, "lifter/left");
    ros::NodeHandle right_lifter_nh(controller_nh, "lifter/right");
    ros::NodeHandle left_wheel_nh(controller_nh, "wheel/left");
    ros::NodeHandle right_wheel_nh(controller_nh, "wheel/right");
    bool success = true;
    success &= left_lifter_controller_.init(hw, left_lifter_nh);
    success &= right_lifter_controller_.init(hw, right_lifter_nh);
    success &= left_wheel_controller_.init(hw, left_wheel_nh);
    success &= right_wheel_controller_.init(hw, right_wheel_nh);
    if (!success) {
      ROS_ERROR("Could not initialize the engineer_lifter_controller (namespace: %s)", controller_nh.getNamespace().c_str());
      return false;
    }
    return true;
  }

  void EngineerLifterController::starting(const ros::Time &time) {
    left_lifter_controller_.starting(time);
    right_lifter_controller_.starting(time);
    left_wheel_controller_.starting(time);
    right_wheel_controller_.starting(time);
  }

  void EngineerLifterController::stopping(const ros::Time &time) {
    left_lifter_controller_.stopping(time);
    right_lifter_controller_.stopping(time);
    left_wheel_controller_.stopping(time);
    right_wheel_controller_.stopping(time);
  }

  void EngineerLifterController::updateControllers(const ros::Time &time, const ros::Duration &period) {
    left_lifter_controller_.update(time, period);
    right_lifter_controller_.update(time, period);
    left_wheel_controller_.update(time, period);
    right_wheel_controller_.update(time, period);
  }

  void EngineerLifterController::update(const ros::Time &time, const ros::Duration &period) {
    switch (current_state) {
      case INIT_STATE:
        left_lifter_controller_.setCommand(0.0);
        right_lifter_controller_.setCommand(0.0);
        left_wheel_controller_.setCommand(0.0);
        right_wheel_controller_.setCommand(0.0);
        updateControllers(time, period);
        break;
      case UP_STATE:
        left_lifter_controller_.setCommand(lift_position_, lift_vel_);
        right_lifter_controller_.setCommand(lift_position_, lift_vel_);
        if (std::abs(left_lifter_controller_.getPosition() - lift_position_) < 0.03 &&
            std::abs(right_lifter_controller_.getPosition() - lift_position_) < 0.03) {
          left_wheel_controller_.setCommand(up_move_vel_);
          right_wheel_controller_.setCommand(up_move_vel_);
        }
        else {
          left_wheel_controller_.setCommand(0.0);
          right_wheel_controller_.setCommand(0.0);
        }
        updateControllers(time, period);
        break;
      case DOWN_READY_STATE:
        left_lifter_controller_.joint_.setCommand(down_ready_effort_);
        right_lifter_controller_.joint_.setCommand(down_ready_effort_);
        break;
    }
  }

  void EngineerLifterController::lifterCmdCallback(const std_msgs::Int32::ConstPtr& msg) {
      current_state = msg->data;
  }
}  // namespace engineer_lifter_controller

PLUGINLIB_EXPORT_CLASS(engineer_lifter_controller::EngineerLifterController, controller_interface::ControllerBase)