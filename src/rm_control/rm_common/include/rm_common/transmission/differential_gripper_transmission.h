//
// Created by ch on 2026/4/29.
//

#pragma once

#include <cassert>
#include <string>
#include <vector>

#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface_exception.h>

namespace transmission_interface
{
class DifferentialGripperTransmission : public Transmission
{
public:
  DifferentialGripperTransmission(const TransmissionInfo& transmission_info,
                                  double roll_act_reduction_,
                                  double grip_act_reduction_,
                                  double roll_jnt_offset_ = 0.0,
                                  double grip_jnt_offset_ = 0.0);
  void actuatorToJointEffort(const ActuatorData& act_data, JointData& jnt_data) override;
  void actuatorToJointVelocity(const ActuatorData& act_data, JointData& jnt_data) override;
  void actuatorToJointPosition(const ActuatorData& act_data, JointData& jnt_data) override;
  void jointToActuatorEffort(const JointData& jnt_data, ActuatorData& act_data) override;
  void jointToActuatorVelocity(const JointData& jnt_data, ActuatorData& act_data) override{};
  void jointToActuatorPosition(const JointData& jnt_data, ActuatorData& act_data) override{};

  std::size_t numActuators() const override
  {
    return 2;
  }
  std::size_t numJoints() const override
  {
    return 2;
  }

protected:
  static double wrap(const double value, const double min, const double max) {
    const double range = max - min;
    double result = fmod(value - min, range);
    if (result < 0) {
      result += range;
    }
    return result;
  }
  double roll_act_reduction_{};
  double grip_act_reduction_{};
  double roll_jnt_offset_{};
  double grip_jnt_offset_{};
};

inline DifferentialGripperTransmission::DifferentialGripperTransmission(const TransmissionInfo& transmission_info,
                                                                        double roll_act_reduction_,
                                                                        double grip_act_reduction_,
                                                                        double roll_jnt_offset_,
                                                                        double grip_jnt_offset_)
  : roll_act_reduction_(roll_act_reduction_), grip_act_reduction_(grip_act_reduction_)
  , roll_jnt_offset_(roll_jnt_offset_), grip_jnt_offset_(grip_jnt_offset_)
{
  if (transmission_info.actuators_.size() != 2)
  {
    throw TransmissionInterfaceException("RollGripperCoupledTransmission requires exactly 2 actuators.");
  }
  if (transmission_info.joints_.size() != 2)
  {
    throw TransmissionInterfaceException("RollGripperCoupledTransmission requires exactly 2 joints.");
  }
  if (roll_act_reduction_ == 0.0 || grip_act_reduction_ == 0.0)
  {
    throw TransmissionInterfaceException("Reduction ratios cannot be zero.");
  }
}
inline void DifferentialGripperTransmission::actuatorToJointEffort(const ActuatorData& act_data, JointData& jnt_data)
{
  const double t1 = *act_data.effort[0]; //roll
  const double t2 = *act_data.effort[1]; //gripper
  *jnt_data.effort[0] = t1 * roll_act_reduction_;
  *jnt_data.effort[1] = t2 * grip_act_reduction_ - t1 * roll_act_reduction_;
}

inline void DifferentialGripperTransmission::actuatorToJointVelocity(const ActuatorData& act_data, JointData& jnt_data)
{
  const double v1 = *act_data.velocity[0];
  const double v2 = *act_data.velocity[1];
  *jnt_data.velocity[0] = v1 / roll_act_reduction_;
  *jnt_data.velocity[1] = v2 / grip_act_reduction_ - v1 / roll_act_reduction_;
}

inline void DifferentialGripperTransmission::actuatorToJointPosition(const ActuatorData& act_data, JointData& jnt_data)
{
  const double p1 = *act_data.position[0];
  const double p2 = *act_data.position[1];
  *jnt_data.position[0] = p1 / roll_act_reduction_ + roll_jnt_offset_;
  *jnt_data.position[1] = wrap(p2 / grip_act_reduction_ + grip_jnt_offset_ - (p1 / roll_act_reduction_ + roll_jnt_offset_), 0.0, 6.28);
}

inline void DifferentialGripperTransmission::jointToActuatorEffort(const JointData& jnt_data, ActuatorData& act_data)
{
  const double roll_t = *jnt_data.effort[0];
  const double grip_t = *jnt_data.effort[1];
  *act_data.effort[0] = roll_t / roll_act_reduction_;
  *act_data.effort[1] = (grip_t + roll_t) / grip_act_reduction_;
}

}  // namespace transmission_interface
