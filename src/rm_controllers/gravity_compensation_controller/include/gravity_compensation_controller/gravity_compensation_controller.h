#pragma once

#include <ros/ros.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <trajectory_interface/trajectory_interface.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include <map>

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

private:
  std::vector<double> gravity_torques_;
  std::vector<double> offset_torques_;
  urdf::Model model_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::map<std::string, hardware_interface::JointStateHandle> joint_states_;
};
} // namespace gravity_compensation_controller

int main(){
  gravity_compensation_controller::GravityCompensationController ctrl;
  
}