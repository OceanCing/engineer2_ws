//
// Created by ch on 2026/4/29.
//

#include "rm_common/transmission/differential_gripper_transmission_loader.h"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/internal/demangle_symbol.h>

namespace transmission_interface
{
TransmissionSharedPtr DifferentialGripperTransmissionLoader::load(const TransmissionInfo& transmission_info)
{
  double roll_act_reduction = 0.0;
  double grip_act_reduction = 0.0;
  double roll_jnt_offset = 0.0;
  double grip_jnt_offset = 0.0;

  if (!getActuatorConfig(transmission_info, roll_act_reduction, grip_act_reduction))
  {
    return TransmissionSharedPtr();
  }

  if (!getJointConfig(transmission_info, roll_jnt_offset, grip_jnt_offset))
  {
    return TransmissionSharedPtr();
  }

  try
  {
    return TransmissionSharedPtr(new DifferentialGripperTransmission(transmission_info,
                                                             roll_act_reduction,
                                                             grip_act_reduction,
                                                             roll_jnt_offset,
                                                             grip_jnt_offset));
  }
  catch (const TransmissionInterfaceException& ex)
  {
    using hardware_interface::internal::demangledTypeName;
    ROS_ERROR_STREAM_NAMED("parser", "Failed to construct transmission '"
                                         << transmission_info.name_ << "' of type '"
                                         << demangledTypeName<DifferentialGripperTransmissionLoader>() << "'. "
                                         << ex.what());
    return TransmissionSharedPtr();
  }
}

bool DifferentialGripperTransmissionLoader::getActuatorConfig(const TransmissionInfo& transmission_info,
                                                      double& roll_act_reduction,
                                                      double& grip_act_reduction)
{
  if (transmission_info.actuators_.size() != 2)
  {
    ROS_ERROR_STREAM_NAMED("parser", "DifferentialGripperTransmission requires exactly 2 actuators.");
    return false;
  }

  TiXmlElement act0 = loadXmlElement(transmission_info.actuators_[0].xml_element_);
  TiXmlElement act1 = loadXmlElement(transmission_info.actuators_[1].xml_element_);

  std::string act0_name = transmission_info.actuators_[0].name_;
  std::string act1_name = transmission_info.actuators_[1].name_;

  getActuatorReduction(act0, act0_name, transmission_info.name_, true, roll_act_reduction);
  getActuatorReduction(act1, act1_name, transmission_info.name_, true, grip_act_reduction);

  return true;
}

bool DifferentialGripperTransmissionLoader::getJointConfig(const TransmissionInfo& transmission_info,
                                                   double& roll_jnt_offset,
                                                   double& grip_jnt_offset)
{
  if (transmission_info.joints_.size() != 2)
  {
    ROS_ERROR_STREAM_NAMED("parser", "DifferentialGripperTransmission requires exactly 2 joints.");
    return false;
  }

  TiXmlElement j0 = loadXmlElement(transmission_info.joints_[0].xml_element_);
  TiXmlElement j1 = loadXmlElement(transmission_info.joints_[1].xml_element_);

  std::string j0_name = transmission_info.joints_[0].name_;
  std::string j1_name = transmission_info.joints_[1].name_;

  getJointOffset(j0, j0_name, transmission_info.name_, false, roll_jnt_offset);
  getJointOffset(j1, j1_name, transmission_info.name_, false, grip_jnt_offset);

  return true;
}

}  // namespace transmission_interface

PLUGINLIB_EXPORT_CLASS(transmission_interface::DifferentialGripperTransmissionLoader,
                       transmission_interface::TransmissionLoader)