#include "gravity_compensation_controller/gravity_compensation_controller.h"
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include "XmlRpcValue.h"
#include "kdl_parser/kdl_parser.hpp"

namespace gravity_compensation_controller
{
bool GravityCompensationController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
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

  KDL::Vector gravity{0.0, 0.0, -9.81};
  solver_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity);

  return true;
}

void GravityCompensationController::update(const ros::Time& time, const ros::Duration& period)
{
  JointTrajectoryController::update(time, period);
  computeGravityTorques();
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    double preset_torque = joints_[i].getCommand();
    joints_[i].setCommand(preset_torque + gravity_torques_[i]);
  }
}

void GravityCompensationController::computeGravityTorques()
{
  int i = 0;
  for (auto& joint : joints_)
  {
    joint_positions_(i) = joint.getPosition();
    ++i;
  }
  KDL::JntArray tau_g;
  solver_->JntToGravity(joint_positions_, tau_g);
  gravity_torques_.resize(tau_g.rows());

  for (int j = 0; j < tau_g.rows(); ++j)
  {
    gravity_torques_[j] = tau_g(j) + (offset_torques_.size() > j ? offset_torques_[j] : 0.0);
  }
}
} // namespace gravity_compensation_controller