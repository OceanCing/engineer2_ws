//
// Created by ch on 2026/2/6.
//

#pragma once

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <effort_controllers/joint_effort_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_position_controller.h>

namespace fix_command_controller {

class FixCommandController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

public:
  FixCommandController() = default;
  bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void starting(const ros::Time &time) override;
  void stopping(const ros::Time &time) override;
  void update(const ros::Time &time, const ros::Duration &period) override;

private:
  effort_controllers::JointEffortController effort_controller_;
  effort_controllers::JointVelocityController velocity_controller_;
  effort_controllers::JointPositionController position_controller_;
  double command_{};
  std::string command_type_;
};

}
