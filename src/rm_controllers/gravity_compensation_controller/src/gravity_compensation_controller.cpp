#include "gravity_compensation_controller/gravity_compensation_controller.h"
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <memory>
#include <XmlRpcValue.h>
#include <pluginlib/class_list_macros.hpp>

namespace gravity_compensation_controller
{
bool GravityCompensationJointTrajectoryController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
  if (!JointTrajectoryController::init(hw, root_nh, controller_nh))
  {
    return false;
  }

  XmlRpc::XmlRpcValue offset_torques;
  controller_nh.param("offset_torques", offset_torques, {0.0});
  if (offset_torques.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Failed to get offset torques, unexpected type.");
    return false;
  }
  for (int i = 0; i < offset_torques.size(); ++i)
  {
    if (offset_torques[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      ROS_ERROR("Failed to get offset torques, unexpected type in array.");
      return false;
    }
    offset_torques_.push_back(static_cast<double>(offset_torques[i]));
  }

  XmlRpc::XmlRpcValue tau_g_scale;
  controller_nh.param("tau_g_scale", tau_g_scale, {0.0});
  if (tau_g_scale.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Failed to get tau g scales, unexpected type.");
    return false;
  }
  for (int i = 0; i < tau_g_scale.size(); ++i)
  {
    if (tau_g_scale[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      ROS_ERROR("Failed to get tau g scales, unexpected type in array.");
      return false;
    }
    tau_g_scale_.push_back(static_cast<double>(tau_g_scale[i]));
  }

  if (!model_.initParam("robot_description"))
  {
    ROS_ERROR("Failed to get robot description.");
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(model_, tree_))
  {
    ROS_ERROR("Failed to parse KDL tree from URDF.");
    return false;
  }

  controller_nh.param("chain_root", chain_root_, std::string("base_link"));
  controller_nh.param("chain_tip", chain_tip_, std::string("gripper_link"));

  if (!tree_.getChain(chain_root_, chain_tip_, chain_))
  {
    ROS_ERROR("Failed to get KDL chain from tree.");
    return false;
  }

  joint_positions_.resize(chain_.getNrOfJoints());
  joint_positions_.data.setZero();
  tau_g_.resize(chain_.getNrOfJoints());
  tau_g_.data.setZero();

  KDL::Vector gravity{0.0, 0.0, -9.81};
  solver_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity);

  pos_hold_server_ = controller_nh.advertiseService("position_hold", &GravityCompensationJointTrajectoryController::positionHoldCallback, this);

  return true;
}

void GravityCompensationJointTrajectoryController::update(const ros::Time& time, const ros::Duration& period)
{
  JointTrajectoryController::update(time, period);
  computeGravityTorques();

  bool desired = position_hold_.load();
  if (desired != last_position_hold_)
  {
    if (desired)
    {
      JointTrajectoryController::starting(time);
      ROS_INFO("GravityCompensationController: hold enabled, command refreshed.");
    }
    last_position_hold_ = desired;
  }
  
  if (desired)
  {
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      double preset_torque = joints_[i].getCommand();
      joints_[i].setCommand(preset_torque + gravity_torques_[i]);
    }
  }
  else
  {
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      joints_[i].setCommand(gravity_torques_[i]);
    }
  }
}

void GravityCompensationJointTrajectoryController::computeGravityTorques()
{
  int idx = 0;
  for (auto& joint : joints_)
  {
    joint_positions_(idx) = joint.getPosition();
    ++idx;
  }

  // Debug
  int ret = solver_->JntToGravity(joint_positions_, tau_g_);
  ROS_INFO_STREAM_THROTTLE(1.0, "JntToGravity ret = " << ret);
  std::ostringstream ss;
  // Debug
  
  gravity_torques_.resize(tau_g_.rows());
  for (int i = 0; i < tau_g_.rows(); ++i)
  {
    gravity_torques_[i] = tau_g_(i) * (tau_g_scale_.size() > i ? tau_g_scale_[i] : 1.0)
      + (offset_torques_.size() > i ? offset_torques_[i] : 0.0);
    // Debug
    ss << gravity_torques_[i] << (i + 1 < tau_g_.rows() ? ", " : "");
    // Debug
  }
  // Debug
  ROS_INFO_THROTTLE(1.0, "Gravity torques: [%s]", ss.str().c_str());
  std::ostringstream qss;
  for (int i = 0; i < joint_positions_.rows(); ++i)
  {
    qss << joint_positions_(i);
    if (i + 1 < joint_positions_.rows())
      qss << ", ";
  }
  ROS_INFO_THROTTLE(1.0, "Joint positions: [%s]", qss.str().c_str());
  // Debug
}

bool GravityCompensationJointTrajectoryController::positionHoldCallback(std_srvs::SetBool::Request& req,
  std_srvs::SetBool::Response& res)
{
  position_hold_.store(req.data);
  res.success = true;
  if(req.data)
    res.message = "Position hold enabled.";
  else
    res.message = "Position hold disabled.";
  return true;
}

} // namespace gravity_compensation_controller

PLUGINLIB_EXPORT_CLASS(gravity_compensation_controller::GravityCompensationJointTrajectoryController, controller_interface::ControllerBase)