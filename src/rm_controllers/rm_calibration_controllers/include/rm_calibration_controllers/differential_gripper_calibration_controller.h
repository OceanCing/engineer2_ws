//
// Created by ch on 2026/4/30.
//

#pragma once
#include "rm_calibration_controllers/calibration_base.h"

namespace rm_calibration_controllers
{
  class DifferentialGripperCalibrationController
    : public CalibrationBase<rm_control::ActuatorExtraInterface, hardware_interface::EffortJointInterface>
  {
  public:
    DifferentialGripperCalibrationController() = default;
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;

  private:
    enum State
    {
      MOVING_POSITIVE = 3,
    };
    int countdown_{};
    double velocity_threshold_{}, actuator_reduction_{};
    ros::Time start_time_{};
    rm_control::ActuatorExtraHandle actuator_gripper_;
    effort_controllers::JointPositionController gripper_position_ctrl_;
  };
}  // namespace rm_calibration_controllers
