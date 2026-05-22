#include "gripper_controller/gripper_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include "ros/node_handle.h"
#include "std_msgs/Bool.h"

namespace gripper_controller {
bool GripperController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  std::string joint_name;
  if (!controller_nh.getParam("joint", joint_name)) {
    ROS_ERROR("Joint was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("max_position", max_position_))
  {
    ROS_ERROR("Max position value was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("min_position", min_position_))
  {
    ROS_ERROR("Min position value was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  joint_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(joint_name);
  ros::NodeHandle opening_nh(controller_nh, "opening");
  ros::NodeHandle closing_nh(controller_nh, "closing");
  opening_pid_.init(ros::NodeHandle(opening_nh, "pid"));
  closing_pid_.init(ros::NodeHandle(closing_nh, "pid"));
  if (!closing_nh.getParam("velocity_threshold", velocity_threshold_))
  {
    ROS_ERROR("Velocity threshold value was not specified (namespace: %s)", closing_nh.getNamespace().c_str());
    return false;
  }
  if (!closing_nh.getParam("position_threshold", position_threshold_))
  {
    ROS_ERROR("Position threshold value was not specified (namespace: %s)", closing_nh.getNamespace().c_str());
    return false;
  }
  if (!closing_nh.getParam("holding_effort", holding_effort_))
  {
    ROS_ERROR("Holding effort value was not specified (namespace: %s)", closing_nh.getNamespace().c_str());
    return false;
  }
  grab_success_ = false;

  cmd_subscriber_ = controller_nh.subscribe<std_msgs::Bool>("command", 1,
                      &GripperController::commandCallback, this);
  is_grabbed_srv_ = controller_nh.advertiseService("grab_state", &GripperController::isGrabbed, this);
  return true;
}

void GripperController::starting(const ros::Time& time){
  state_ = INITIALIZED;
  ROS_INFO("Initialized GripperController");
};

void GripperController::stopping(const ros::Time& time){
  grab_success_ = false;
};

void GripperController::update(const ros::Time& time, const ros::Duration& period){
  double cmd_joint = 0.0;
  switch (state_)
  {
    case INITIALIZED:
    {
      state_ = OPENING;
      break;
    }
    case OPENING:
    {
      cmd_joint = opening_pid_.computeCommand(max_position_ - joint_.getPosition(), period);

      if(cmd_gripper_ == true){
        state_ = CLOSING;
        countdown_ = 50;
      }
      break;
    }
    case CLOSING:
    {
      cmd_joint = closing_pid_.computeCommand(min_position_ - joint_.getPosition(), period);
      if (std::abs(joint_.getVelocity()) < velocity_threshold_)
        countdown_--;
      else
        countdown_ = 50;
      if (countdown_ < 0)
      {
        state_ = HOLDING;
        grab_success_ = true;
      }
      if(cmd_gripper_ == false){
        state_ = OPENING;
        grab_success_ = false;
      }
      break;
    }
    case HOLDING:
    {
      // if (abs(min_position_ - joint_.getPosition()) <= position_threshold_) {
      //   cmd_joint = 0.0;
      // }
      // else
      //   cmd_joint = holding_effort_;
      cmd_joint = holding_effort_;
      if(cmd_gripper_ == false){
        state_ = OPENING;
        grab_success_ = false;
      }
      break;
    }
  }
  joint_.setCommand(cmd_joint);
}

void GripperController::commandCallback(const std_msgs::Bool::ConstPtr& msg){
  cmd_gripper_ = msg->data;
}

bool GripperController::isGrabbed(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
  res.success = grab_success_;
  res.message = "Gripper's grabbing state.";
  return true;
}

} // namespace gripper_controller

PLUGINLIB_EXPORT_CLASS(gripper_controller::GripperController, controller_interface::ControllerBase)