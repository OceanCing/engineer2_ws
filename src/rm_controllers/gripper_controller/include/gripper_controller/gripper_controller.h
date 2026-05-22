#pragma once

#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_effort_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <std_msgs/Bool.h>

#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/hardware_interface/gpio_interface.h>
#include "ros/subscriber.h"
#include "std_srvs/Trigger.h"
#include <dynamic_reconfigure/server.h>
#include <memory>

namespace gripper_controller
{

  class GripperController
    : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
  {
  public:
    GripperController() = default;

    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

    void starting(const ros::Time& time) override;
    void stopping(const ros::Time& time) override;

    void update(const ros::Time& time, const ros::Duration& period) override;

    void commandCallback(const std_msgs::Bool::ConstPtr& msg);
    bool isGrabbed(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res);
  private:
    enum State
    {
      INITIALIZED,
      OPENING,
      CLOSING,
      HOLDING
    };
    int state_{};
    int countdown_{};
    double max_position_{}, min_position_{};
    double velocity_threshold_{}, position_threshold_{};
    double holding_effort_{};
    bool grab_success_ = false;
    bool cmd_gripper_ = false;
    hardware_interface::JointHandle joint_;
    control_toolbox::Pid opening_pid_, closing_pid_;

    ros::Subscriber cmd_subscriber_;
    ros::ServiceServer is_grabbed_srv_;
  };

}  // namespace gripper_controller