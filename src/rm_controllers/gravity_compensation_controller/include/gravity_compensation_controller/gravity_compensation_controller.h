#pragma once

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <trajectory_interface/trajectory_interface.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <urdf/model.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>

#include <map>
#include <atomic>

namespace gravity_compensation_controller
{
class GravityCompensationJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController<
  trajectory_interface::QuinticSplineSegment<double>,
  hardware_interface::EffortJointInterface>
{
public:
  GravityCompensationJointTrajectoryController() : JointTrajectoryController() {}

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  bool positionHoldCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

private:
  void computeGravityTorques();

  std::vector<double> gravity_torques_;
  std::vector<double> offset_torques_;
  std::vector<double> tau_g_scale_;
  std::string chain_root_,chain_tip_;
  urdf::Model model_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> solver_;
  KDL::JntArray joint_positions_;
  KDL::JntArray tau_g_;
  std::atomic<bool> position_hold_{false};
  bool last_position_hold_ = false;
  ros::ServiceServer pos_hold_server_;
  // std::map<std::string, hardware_interface::JointStateHandle> joint_states_;
};
} // namespace gravity_compensation_controller