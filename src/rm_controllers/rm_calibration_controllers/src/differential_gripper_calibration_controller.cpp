//
// Created by ch on 2026/4/30.
//

#include "rm_calibration_controllers/differential_gripper_calibration_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace rm_calibration_controllers
{
bool DifferentialGripperCalibrationController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                           ros::NodeHandle& controller_nh)
{
  CalibrationBase::init(robot_hw, root_nh, controller_nh);
  ros::NodeHandle pos_nh(controller_nh, "position");
  position_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), pos_nh);
  XmlRpc::XmlRpcValue actuator_gripper;
  if (!controller_nh.getParam("actuator_gripper", actuator_gripper))
  {
    ROS_ERROR("No actuator_roll given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  actuator_gripper_ = robot_hw->get<rm_control::ActuatorExtraInterface>()->getHandle(actuator_gripper[0]);
  if (!controller_nh.getParam("actuator_reduction", actuator_reduction_))
  {
    ROS_ERROR("Actuator reduction was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("velocity/velocity_threshold", velocity_threshold_))
  {
    ROS_ERROR("Velocity threshold was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (velocity_threshold_ < 0)
  {
    velocity_threshold_ *= -1.;
    ROS_ERROR("Negative velocity threshold is not supported for joint %s. Making the velocity threshold positive.",
              velocity_ctrl_.getJointName().c_str());
  }
  return true;
}

void DifferentialGripperCalibrationController::update(const ros::Time& time, const ros::Duration& period)
{
  switch (state_)
  {
    case INITIALIZED:
    {
      velocity_ctrl_.joint_.setCommand(velocity_search_);
      position_ctrl_.setCommand(position_ctrl_.getPosition());
      countdown_ = 100;
      state_ = MOVING_POSITIVE;
      start_time_ = time;
      break;
    }
    case MOVING_POSITIVE:
    {
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) < velocity_threshold_ && !actuator_gripper_.getHalted())
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ < 0)
      {
        // velocity_ctrl_.setCommand(0);
        actuator_.setOffset(-actuator_.getPosition() + actuator_gripper_.getPosition() * actuator_reduction_ + actuator_.getOffset());
        actuator_.setCalibrated(true);
        ROS_INFO("Actuator %s calibrated", actuator_.getName().c_str());
        state_ = CALIBRATED;
        velocity_ctrl_.joint_.setCommand(0.);
        position_ctrl_.setCommand(0.);
        calibration_success_ = true;
      }
      position_ctrl_.update(time, period);
      if (time - start_time_ > ros::Duration(0.8))
        velocity_ctrl_.joint_.setCommand(0.0);
      else
        velocity_ctrl_.joint_.setCommand(velocity_search_);
      break;
    }
    case CALIBRATED:
    {
      position_ctrl_.update(time, period);
      // velocity_ctrl_.update(time, period);
      break;
    }
  }
}
}  // namespace rm_calibration_controllers

PLUGINLIB_EXPORT_CLASS(rm_calibration_controllers::DifferentialGripperCalibrationController,
                       controller_interface::ControllerBase)