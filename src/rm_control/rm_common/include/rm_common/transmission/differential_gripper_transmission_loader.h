//
// Created by ch on 2026/4/29.
//

#pragma once

#include <tinyxml.h>
#include <transmission_interface/transmission_loader.h>
#include "rm_common/transmission/differential_gripper_transmission.h"

namespace transmission_interface
{
class DifferentialGripperTransmissionLoader : public TransmissionLoader
{
public:
  TransmissionSharedPtr load(const TransmissionInfo& transmission_info) override;

private:
  static bool getActuatorConfig(const TransmissionInfo& transmission_info,
                                double& roll_act_reduction,
                                double& grip_act_reduction);

  static bool getJointConfig(const TransmissionInfo& transmission_info,
                              double& roll_jnt_offset,
                              double& grip_jnt_offset);
};

}  // namespace transmission_interface