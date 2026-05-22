//
// Created by ch on 2026/3/24.
//

#pragma once

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <std_msgs/Int32.h>

namespace engineer_lifter_controller {

class EngineerLifterController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

public:
  EngineerLifterController() = default;
  bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void starting(const ros::Time &time) override;
  void stopping(const ros::Time &time) override;
  void update(const ros::Time &time, const ros::Duration &period) override;

private:
  enum {
    INIT_STATE,
    UP_STATE,
    DOWN_READY_STATE,
    DOWN_MOVE_STATE
  };
  void lifterCmdCallback(const std_msgs::Int32::ConstPtr& msg);
  void updateControllers(const ros::Time &time, const ros::Duration &period);
  int current_state = INIT_STATE;
  double lift_position_, lift_vel_, up_move_vel_, down_ready_effort_;
  effort_controllers::JointVelocityController left_wheel_controller_, right_wheel_controller_;
  effort_controllers::JointPositionController left_lifter_controller_, right_lifter_controller_;
  ros::Subscriber cmd_sub_;
};
}  // namespace engineer_lifter_controller
