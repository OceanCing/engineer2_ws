#include "gravity_compensation_controller/gravity_compensation_controller.h"
#include <kdl/chaindynparam.hpp>
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

  KDL::Vector gravity{0.0, 0.0, -9.81};
  solver_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity);

  return true;
}

void GravityCompensationController::update(const ros::Time& time, const ros::Duration& period){}

void GravityCompensationController::computeGravityTorques()
{
  joint_positions_.resize(chain_.getNrOfJoints());
  for (size_t i = 0; i < chain_.getNrOfJoints(); ++i)
  {
    joint_positions_(i) = joints_[i].getPosition();
  }

}
} // namespace gravity_compensation_controller