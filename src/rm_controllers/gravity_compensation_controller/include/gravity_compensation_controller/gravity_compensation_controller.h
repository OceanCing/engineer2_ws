#pragma once

#include <ros/ros.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "trajectory_interface/trajectory_interface.h"
#include <trajectory_interface/quintic_spline_segment.h>

namespace gravity_compensation_controller
{
class GravityCompensationController : public joint_trajectory_controller::JointTrajectoryController<
  trajectory_interface::QuinticSplineSegment<double>,
  hardware_interface::EffortJointInterface>
{
public:
  GravityCompensationController() = default;

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;
};
}

int main(){
  gravity_compensation_controller::GravityCompensationController ctrl;
  
}