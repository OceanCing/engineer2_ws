//
// Created by qiayuan on 5/23/21.
//

#pragma once

#include "chassis_gimbal_manual.h"

#include <utility>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <actionlib/client/simple_action_client.h>
#include <rm_common/decision/calibration_queue.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rm_msgs/EngineerAction.h>
#include <rm_msgs/MultiDofCmd.h>
#include <rm_msgs/GpioData.h>
#include <rm_msgs/EngineerUi.h>
#include <rm_msgs/VisualizeStateData.h>
#include <rm_msgs/CustomControllerData.h>
#include <rm_msgs/VTReceiverControlData.h>
#include <control_msgs/JointJog.h>
#include <control_toolbox/pid.h>
#include <stack>
#include "unordered_map"

namespace rm_manual
{
class Engineer2Manual : public ChassisGimbalManual
{
public:
  enum ControlMode
  {
    MANUAL,
    MIDDLEWARE
  };

  enum JointMode
  {
    SERVO,
    JOINT,
    CUSTOM,
    VT_CONTROL
  };

  enum GimbalMode
  {
    RATE,
    DIRECT
  };

  enum SpeedMode
  {
    LOW,
    NORMAL,
    FAST,
    EXCHANGE,
    BIG_ISLAND_SPEED
  };

  enum SynchStatus
  {
    WITHOUT_SYNCH,
    SYNCHRONIZING,
    IN_SYNCH
  };

  Engineer2Manual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  void run() override;

private:
  void changeSpeedMode(SpeedMode speed_mode);
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updateServo(const rm_msgs::VTReceiverControlData::ConstPtr& vt_control_data);
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override;
  void gpioStateCallback(const rm_msgs::GpioData::ConstPtr& data);
  void sendCommand(const ros::Time& time) override;
  void runStepQueue(const std::string& step_queue_name);
  void actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr& feedback);
  void actionDoneCallback(const actionlib::SimpleClientGoalState& state, const rm_msgs::EngineerResultConstPtr& result);
  void customControllerCallback(const rm_msgs::CustomControllerData::ConstPtr& custom_data);
  void updateCustomController(const rm_msgs::CustomControllerData::ConstPtr& custom_data);
  void checkCustomButton(const rm_msgs::CustomControllerData::ConstPtr& custom_data);
  void vtControlCallback(const rm_msgs::VTReceiverControlData::ConstPtr& vt_control_data);
  void updateVTControl(const rm_msgs::VTReceiverControlData::ConstPtr& vt_control_data);
  void distSensorCallback(const std_msgs::Float64::ConstPtr& angle_data);

  void initMode();
  void enterServo();
  void enterCustom();
  void actionActiveCallback()
  {
    operating_mode_ = MIDDLEWARE;
  }
  void remoteControlTurnOff() override;
  void chassisOutputOn() override;
  void gimbalOutputOn() override;

  void rightSwitchUpRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchDownRise() override;
  void leftSwitchUpRise() override;
  void leftSwitchUpFall();
  void leftSwitchDownRise() override;
  void leftSwitchDownFall();
  void ctrlAPress();
  void ctrlBPress();
  void ctrlBPressing();
  void ctrlBRelease();
  void ctrlCPress();
  void ctrlDPress();
  void ctrlEPress();
  void ctrlFPress();
  void ctrlGPress();
  void ctrlQPress();
  void ctrlRPress();
  void ctrlSPress();
  void ctrlVPress();
  void ctrlVRelease();
  void ctrlWPress();
  void ctrlXPress();
  void ctrlZPress();

  // void wPressing() override;
  // void aPressing() override;
  // void sPressing() override;
  // void dPressing() override;
  void bPressing();
  void bRelease();
  void cPressing();
  void cRelease();
  void ePressing();
  void eRelease();
  void fPress();
  void fRelease();
  void gPress();
  void gRelease();
  void qPressing();
  void qRelease();
  void rPress();
  void rRelease();
  void vPressing();
  void vRelease();
  void xPress();
  void zPressing();
  void zRelease();
  void zPress();

  void shiftPressing();
  void shiftRelease();
  void shiftBPress();
  void shiftBRelease();
  void shiftCPress();
  void shiftEPress();
  void shiftFPress();
  void shiftGPress();
  void shiftQPress();
  void shiftRPress();
  void shiftRRelease();
  void shiftVPress();
  void shiftVRelease();
  void shiftXPress();
  void shiftZPress();
  void shiftZRelease();

  void customButton1Press();
  void customButton2Press();
  void customButton3Press();
  void customButton4Press();

  void mouseLeftRelease();
  void mouseRightRelease();

  void vtModeC();
  void vtModeN();
  void vtModeSRise();
  void vtModeSFall();
  void vtCustomButtonLPress();
  void vtCustomButtonLRelease();
  void vtCustomButtonRPress();
  void vtCustomButtonRRelease();
  // void vtPauseButtonPress();
  // void vtPauseButtonRelease();
  // void vtTriggerPress();
  // void vtTriggerRelease();
  // Servo
  std::vector<std::string> joint_servo_joint_names_{};
  std::vector<double> joint_servo_max_vel_{};
  ros::Publisher joint_jog_pub_, servo_cmd_pub_;
  control_msgs::JointJog joint_jog_cmd_;
  geometry_msgs::TwistStamped servo_cmd_;
  double joint1_scale_{}, joint7_scale_{};
  double linear_z_{};

  bool mouse_left_pressed_{}, mouse_right_pressed_{}, had_ground_stone_{ false }, small_gripper_on_{ false },
      had_side_gold_{ false }, stone_state_[4]{}, is_first_stone_{ true }, is_island_{ false }, need_transfer_{ false }, chassis_locked_{ false }, big_island_right_{ false };
  double angular_z_scale_{}, gyro_scale_{}, fast_gyro_scale_{}, low_gyro_scale_{}, normal_gyro_scale_{},
      exchange_gyro_scale_{}, fast_speed_scale_{}, low_speed_scale_{}, normal_speed_scale_{}, exchange_speed_scale_{},joint1_speed_scale_{0.1},
      big_island_speed_scale_{}, custom_data_offset_[6]{}, custom2joint_offset_[6]{}, yaw_lock_error_{}, variable_joint4_limit_{};

  std::string prefix_{}, root_{}, exchange_direction_{ "left" }, exchange_arm_position_{ "normal" };
  int operating_mode_{}, control_mode_{ 1 }, gimbal_mode_{}, gimbal_direction_{ 0 }, custom_seq_{ 1 }, small_arm_stone_{ 0 },
      custom2joint_orientation_[6]{}, custom_joystick_offset_[4]{}, custom_joystick_dead_zone_{ 0 }, is_in_synch_{ 0 };

  std::stack<std::string> stone_num_{};

  ros::Subscriber gripper_state_sub_, custom_controller_sub_, dist_sensor_sub_;
  ros::Publisher engineer_ui_pub_, gripper_ui_pub_, trajectory_cmd_pub_;
  ros::ServiceClient gripper_state_caller_;

  ros::Subscriber vt_control_data_sub_;

  ros::Time start_synch_time_, custom_last_;

  rm_msgs::GpioData gpio_state_;
  rm_msgs::EngineerUi engineer_ui_, old_ui_;
  rm_msgs::VisualizeStateData gripper_ui_;

  rm_common::Vel3DCommandSender* servo_command_sender_;
  rm_common::ServiceCallerBase<std_srvs::Empty>* servo_reset_caller_;
  rm_common::CalibrationQueue* calibration_gather_{};

  control_toolbox::Pid yaw_lock_pid_;

  actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;

  InputEvent left_switch_up_event_, left_switch_down_event_, ctrl_a_event_, ctrl_b_event_, ctrl_c_event_, ctrl_d_event_,
      ctrl_e_event_, ctrl_f_event_, ctrl_g_event_, ctrl_q_event_, ctrl_r_event_, ctrl_s_event_, ctrl_v_event_,
      ctrl_w_event_, ctrl_x_event_, ctrl_z_event_, b_event_, c_event_, e_event_, f_event_, g_event_, q_event_, r_event_,
      v_event_, x_event_, z_event_, shift_event_, shift_b_event_, shift_c_event_, shift_e_event_, shift_f_event_,
      shift_g_event_, shift_v_event_, shift_q_event_, shift_r_event_, shift_x_event_, shift_z_event_, mouse_left_event_,
      mouse_right_event_, custom_button1_event_, custom_button2_event_, custom_button3_event_, custom_button4_event_;
  InputEvent vt_trigger_event_, vt_custom_button_l_event_, vt_custom_button_r_event_,
      vt_pause_button_event_,vt_mode_c_event_, vt_mode_n_event_, vt_mode_s_event_;
  bool vt_trigger_pressed_;
  ros::Time pause_pressed_;


  enum UiState
  {
    NONE,
    BIG_ISLAND,
    SMALL_ISLAND
  };

  enum StoneType
  {
    EMPTY,
    GOLD,
    SILVER
  };
};

}  // namespace rm_manual
