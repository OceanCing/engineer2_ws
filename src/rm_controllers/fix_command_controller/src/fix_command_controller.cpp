//
// Created by ch on 2026/2/6.
//

#include "fix_command_controller/fix_command_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace fix_command_controller {
  bool FixCommandController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    if (!controller_nh.getParam("command_type", command_type_)) {
      ROS_ERROR("Command type was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
      return false;
    }
    ros::NodeHandle ctrl_param_nh(controller_nh, "controller");
    if (command_type_ == "effort") {
      effort_controller_.init(hw, ctrl_param_nh);
      effort_controller_.state_ = ControllerState::INITIALIZED;
    } else if (command_type_ == "velocity") {
      velocity_controller_.init(hw, ctrl_param_nh);
      velocity_controller_.state_ = ControllerState::INITIALIZED;
    } else if (command_type_ == "position") {
      position_controller_.init(hw, ctrl_param_nh);
      position_controller_.state_ = ControllerState::INITIALIZED;
    } else {
      ROS_ERROR("Command type error (effort/velocity/position) (namespace: %s)", ctrl_param_nh.getNamespace().c_str());
      return false;
    }
    if (!controller_nh.getParam("command", command_)) {
      ROS_WARN("Command was not specified, use default value 0.0 (namespace: %s)", controller_nh.getNamespace().c_str());
    }
    return true;
  }

  void FixCommandController::starting(const ros::Time &time) {
    if (effort_controller_.isInitialized()) {
      if (effort_controller_.startRequest(time))
        effort_controller_.command_buffer_.writeFromNonRT(command_);
    } else if (velocity_controller_.isInitialized()) {
      if (velocity_controller_.startRequest(time))
        velocity_controller_.setCommand(command_);
    } else if (position_controller_.isInitialized()) {
      if (position_controller_.startRequest(time))
        position_controller_.setCommand(command_);
    }
  }

  void FixCommandController::stopping(const ros::Time &time) {
    if (effort_controller_.isInitialized())
      effort_controller_.stopRequest(time);
    else if (velocity_controller_.isInitialized())
      velocity_controller_.stopRequest(time);
    else if (position_controller_.isInitialized())
      position_controller_.stopRequest(time);
  }

  void FixCommandController::update(const ros::Time &time, const ros::Duration &period) {
    effort_controller_.updateRequest(time, period);
    velocity_controller_.updateRequest(time, period);
    position_controller_.updateRequest(time, period);
  }

} // namespace fix_command_controller
PLUGINLIB_EXPORT_CLASS(fix_command_controller::FixCommandController, controller_interface::ControllerBase)
