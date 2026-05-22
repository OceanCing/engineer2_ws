//
// Created by cch on 24-5-31.
//
#include "rm_manual/engineer2_manual.h"

namespace rm_manual
{
Engineer2Manual::Engineer2Manual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : ChassisGimbalManual(nh, nh_referee)
  , operating_mode_(MANUAL)
  , action_client_("/engineer_middleware/move_steps", true)
{
  engineer_ui_pub_ = nh.advertise<rm_msgs::EngineerUi>("/engineer_ui", 10);
  ROS_INFO("Waiting for middleware to start.");
  action_client_.waitForServer();
  ROS_INFO("Middleware started.");
  gripper_state_sub_ = nh.subscribe<rm_msgs::GpioData>("/controllers/gpio_controller/gpio_states", 10,
                                                       &Engineer2Manual::gpioStateCallback, this);
  gripper_ui_pub_ = nh.advertise<rm_msgs::VisualizeStateData>("/visualize_state", 10);
  gripper_state_caller_ = nh.serviceClient<std_srvs::Trigger>("/controllers/gripper_controller/grab_state");
  custom_controller_sub_ = nh.subscribe<rm_msgs::CustomControllerData>("/rm_vt/custom_controller_data", 10, &Engineer2Manual::customControllerCallback,this);
  dist_sensor_sub_ = nh.subscribe<std_msgs::Float64>("/dist_sensor/yaw_angle", 10, &Engineer2Manual::distSensorCallback, this);
  trajectory_cmd_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/controllers/arm_trajectory_controller/command", 10);
  vt_control_data_sub_ = nh.subscribe<rm_msgs::VTReceiverControlData>("/rm_vt/receiver_control_data", 1, &Engineer2Manual::vtControlCallback, this);
  // Servo
  ros::NodeHandle nh_servo(nh, "servo");
  servo_command_sender_ = new rm_common::Vel3DCommandSender(nh_servo);
  servo_reset_caller_ = new rm_common::ServiceCallerBase<std_srvs::Empty>(nh_servo, "/servo_server/reset_servo_status");
  // Joint Servo
  ros::NodeHandle nh_joint_servo(nh, "joint_servo");
  std::string joint_topic;
  nh_joint_servo.param("topic", joint_topic, std::string("/servo_server/delta_joint_cmds"));
  XmlRpc::XmlRpcValue joint_names;
  if (nh_joint_servo.getParam("joint_names", joint_names) && joint_names.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < joint_names.size(); ++i)
      joint_servo_joint_names_.push_back(static_cast<std::string>(joint_names[i]));
  }
  XmlRpc::XmlRpcValue max_vels;
  if (nh_joint_servo.getParam("max_vel", max_vels) && max_vels.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < max_vels.size(); ++i)
      joint_servo_max_vel_.push_back(static_cast<double>(max_vels[i]));
  }
  joint_jog_pub_ = nh.advertise<control_msgs::JointJog>(joint_topic, 10);
  servo_cmd_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 10);
  // Custom Controller
  ros::NodeHandle nh_custom_controller(nh, "custom_controller");
  if (nh_custom_controller.hasParam("custom_data_offset") && nh_custom_controller.hasParam("custom2joint_offset")
    && nh_custom_controller.hasParam("custom2joint_orientation") && nh_custom_controller.hasParam("custom_joystick_offset")
    && nh_custom_controller.hasParam("custom_joystick_dead_zone") && nh_custom_controller.hasParam("custom_joint1_speed_scare")
    && nh_custom_controller.hasParam("variable_joint4_limit"))
  {
    XmlRpc::XmlRpcValue xml_rpc_value;
    nh_custom_controller.getParam("custom_data_offset",xml_rpc_value);
    for(int i = 0; i < xml_rpc_value.size(); i++)
      custom_data_offset_[i] = xml_rpc_value[i];
    nh_custom_controller.getParam("custom2joint_offset",xml_rpc_value);
    for(int i = 0; i < xml_rpc_value.size(); i++)
      custom2joint_offset_[i] = xml_rpc_value[i];
    nh_custom_controller.getParam("custom2joint_orientation",xml_rpc_value);
    for(int i = 0; i < xml_rpc_value.size(); i++)
      custom2joint_orientation_[i] = xml_rpc_value[i];
    nh_custom_controller.getParam("custom_joystick_offset",xml_rpc_value);
    for(int i = 0; i < xml_rpc_value.size(); i++)
      custom_joystick_offset_[i] = xml_rpc_value[i];
    nh_custom_controller.getParam("custom_joystick_dead_zone",custom_joystick_dead_zone_);
    nh_custom_controller.getParam("custom_joint1_speed_scare",joint1_speed_scale_);
    nh_custom_controller.getParam("variable_joint4_limit",variable_joint4_limit_);
  }
  yaw_lock_pid_.init(ros::NodeHandle(nh_custom_controller, "yaw_lock_pid"));
  // Vel
  ros::NodeHandle chassis_nh(nh, "chassis");
  chassis_nh.param("fast_speed_scale", fast_speed_scale_, 1.0);
  chassis_nh.param("normal_speed_scale", normal_speed_scale_, 0.5);
  chassis_nh.param("low_speed_scale", low_speed_scale_, 0.1);
  chassis_nh.param("exchange_speed_scale", exchange_speed_scale_, 0.2);
  chassis_nh.param("big_island_speed_scale", big_island_speed_scale_, 0.02);
  chassis_nh.param("fast_gyro_scale", fast_gyro_scale_, 0.5);
  chassis_nh.param("normal_gyro_scale", normal_gyro_scale_, 0.15);
  chassis_nh.param("low_gyro_scale", low_gyro_scale_, 0.05);
  chassis_nh.param("exchange_gyro_scale", exchange_gyro_scale_, 0.12);
  // Calibration
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("calibration_gather", rpc_value);
  calibration_gather_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  left_switch_up_event_.setFalling(boost::bind(&Engineer2Manual::leftSwitchUpFall, this));
  left_switch_up_event_.setRising(boost::bind(&Engineer2Manual::leftSwitchUpRise, this));
  left_switch_down_event_.setFalling(boost::bind(&Engineer2Manual::leftSwitchDownFall, this));
  left_switch_down_event_.setRising(boost::bind(&Engineer2Manual::leftSwitchDownRise, this));
  ctrl_a_event_.setRising(boost::bind(&Engineer2Manual::ctrlAPress, this));
  ctrl_b_event_.setRising(boost::bind(&Engineer2Manual::ctrlBPress, this));
  ctrl_c_event_.setRising(boost::bind(&Engineer2Manual::ctrlCPress, this));
  ctrl_b_event_.setActiveHigh(boost::bind(&Engineer2Manual::ctrlBPressing, this));
  ctrl_b_event_.setFalling(boost::bind(&Engineer2Manual::ctrlBRelease, this));
  ctrl_d_event_.setRising(boost::bind(&Engineer2Manual::ctrlDPress, this));
  ctrl_e_event_.setRising(boost::bind(&Engineer2Manual::ctrlEPress, this));
  ctrl_f_event_.setRising(boost::bind(&Engineer2Manual::ctrlFPress, this));
  ctrl_g_event_.setRising(boost::bind(&Engineer2Manual::ctrlGPress, this));
  ctrl_q_event_.setRising(boost::bind(&Engineer2Manual::ctrlQPress, this));
  ctrl_r_event_.setRising(boost::bind(&Engineer2Manual::ctrlRPress, this));
  ctrl_s_event_.setRising(boost::bind(&Engineer2Manual::ctrlSPress, this));
  ctrl_v_event_.setRising(boost::bind(&Engineer2Manual::ctrlVPress, this));
  ctrl_v_event_.setFalling(boost::bind(&Engineer2Manual::ctrlVRelease, this));
  ctrl_w_event_.setRising(boost::bind(&Engineer2Manual::ctrlWPress, this));
  ctrl_x_event_.setRising(boost::bind(&Engineer2Manual::ctrlXPress, this));
  ctrl_z_event_.setRising(boost::bind(&Engineer2Manual::ctrlZPress, this));
  b_event_.setActiveHigh(boost::bind(&Engineer2Manual::bPressing, this));
  b_event_.setFalling(boost::bind(&Engineer2Manual::bRelease, this));
  c_event_.setActiveHigh(boost::bind(&Engineer2Manual::cPressing, this));
  c_event_.setFalling(boost::bind(&Engineer2Manual::cRelease, this));
  e_event_.setActiveHigh(boost::bind(&Engineer2Manual::ePressing, this));
  e_event_.setFalling(boost::bind(&Engineer2Manual::eRelease, this));
  f_event_.setRising(boost::bind(&Engineer2Manual::fPress, this));
  f_event_.setFalling(boost::bind(&Engineer2Manual::fRelease, this));
  g_event_.setRising(boost::bind(&Engineer2Manual::gPress, this));
  g_event_.setFalling(boost::bind(&Engineer2Manual::gRelease, this));
  q_event_.setActiveHigh(boost::bind(&Engineer2Manual::qPressing, this));
  q_event_.setFalling(boost::bind(&Engineer2Manual::qRelease, this));
  r_event_.setRising(boost::bind(&Engineer2Manual::rPress, this));
  r_event_.setFalling(boost::bind(&Engineer2Manual::rRelease, this));
  v_event_.setActiveHigh(boost::bind(&Engineer2Manual::vPressing, this));
  v_event_.setFalling(boost::bind(&Engineer2Manual::vRelease, this));
  x_event_.setRising(boost::bind(&Engineer2Manual::xPress, this));
  z_event_.setActiveHigh(boost::bind(&Engineer2Manual::zPressing, this));
  z_event_.setFalling(boost::bind(&Engineer2Manual::zRelease, this));
  z_event_.setRising(boost::bind(&Engineer2Manual::zPress, this));
  shift_event_.setActiveHigh(boost::bind(&Engineer2Manual::shiftPressing, this));
  shift_event_.setFalling(boost::bind(&Engineer2Manual::shiftRelease, this));
  shift_b_event_.setRising(boost::bind(&Engineer2Manual::shiftBPress, this));
  shift_b_event_.setFalling(boost::bind(&Engineer2Manual::shiftBRelease, this));
  shift_c_event_.setRising(boost::bind(&Engineer2Manual::shiftCPress, this));
  shift_e_event_.setRising(boost::bind(&Engineer2Manual::shiftEPress, this));
  shift_f_event_.setRising(boost::bind(&Engineer2Manual::shiftFPress, this));
  shift_g_event_.setRising(boost::bind(&Engineer2Manual::shiftGPress, this));
  shift_q_event_.setRising(boost::bind(&Engineer2Manual::shiftQPress, this));
  shift_r_event_.setRising(boost::bind(&Engineer2Manual::shiftRPress, this));
  shift_r_event_.setFalling(boost::bind(&Engineer2Manual::shiftRRelease, this));
  shift_v_event_.setRising(boost::bind(&Engineer2Manual::shiftVPress, this));
  shift_v_event_.setFalling(boost::bind(&Engineer2Manual::shiftVRelease, this));
  shift_x_event_.setRising(boost::bind(&Engineer2Manual::shiftXPress, this));
  shift_z_event_.setRising(boost::bind(&Engineer2Manual::shiftZPress, this));
  shift_z_event_.setFalling(boost::bind(&Engineer2Manual::shiftZRelease, this));

  mouse_left_event_.setFalling(boost::bind(&Engineer2Manual::mouseLeftRelease, this));
  mouse_right_event_.setFalling(boost::bind(&Engineer2Manual::mouseRightRelease, this));

  vt_mode_c_event_.setRising(boost::bind(&Engineer2Manual::vtModeC, this));
  vt_mode_c_event_.setActiveHigh(boost::bind(&Engineer2Manual::vtModeC, this));
  vt_mode_n_event_.setRising(boost::bind(&Engineer2Manual::vtModeN, this));
  vt_mode_n_event_.setActiveHigh(boost::bind(&Engineer2Manual::vtModeN, this));
  vt_mode_s_event_.setRising(boost::bind(&Engineer2Manual::vtModeSRise, this));
  vt_mode_s_event_.setFalling(boost::bind(&Engineer2Manual::vtModeSFall, this));
  vt_custom_button_l_event_.setRising(boost::bind(&Engineer2Manual::vtCustomButtonLPress, this));
  vt_custom_button_l_event_.setFalling(boost::bind(&Engineer2Manual::vtCustomButtonLRelease, this));
  vt_custom_button_r_event_.setRising(boost::bind(&Engineer2Manual::vtCustomButtonRPress, this));
  vt_custom_button_r_event_.setFalling(boost::bind(&Engineer2Manual::vtCustomButtonRRelease, this));
  // vt_pause_button_event_.setRising(boost::bind(&Engineer2Manual::vtPauseButtonPress, this));
  // vt_pause_button_event_.setFalling(boost::bind(&Engineer2Manual::vtPauseButtonRelease, this));
  // vt_trigger_event_.setRising(boost::bind(&Engineer2Manual::vtTriggerPress, this));
  // vt_trigger_event_.setFalling(boost::bind(&Engineer2Manual::vtTriggerRelease, this));

  custom_button1_event_.setRising(boost::bind(&Engineer2Manual::customButton1Press, this));
  custom_button2_event_.setRising(boost::bind(&Engineer2Manual::customButton2Press, this));
  custom_button3_event_.setRising(boost::bind(&Engineer2Manual::customButton3Press, this));
  custom_button4_event_.setRising(boost::bind(&Engineer2Manual::customButton4Press, this));
}

void Engineer2Manual::run()
{
  ChassisGimbalManual::run();
  calibration_gather_->update(ros::Time::now());
  if (joint_state_.position.size() > 0)
  {
    engineer_ui_.joint1_position = joint_state_.position[0];
    engineer_ui_.yaw_position = joint_state_.position[15] + joint_state_.position[1];
  }
  engineer_ui_pub_.publish(engineer_ui_);
  gripper_ui_pub_.publish(gripper_ui_);
}

void Engineer2Manual::changeSpeedMode(SpeedMode speed_mode)
{
  switch (speed_mode)
  {
    case LOW:
      speed_change_scale_ = low_speed_scale_;
      gyro_scale_ = low_gyro_scale_;
      break;
    case NORMAL:
      speed_change_scale_ = normal_speed_scale_;
      gyro_scale_ = normal_gyro_scale_;
      break;
    case FAST:
      speed_change_scale_ = fast_speed_scale_;
      gyro_scale_ = fast_gyro_scale_;
      break;
    case EXCHANGE:
      speed_change_scale_ = exchange_speed_scale_;
      gyro_scale_ = exchange_gyro_scale_;
      break;
    case BIG_ISLAND_SPEED:
      speed_change_scale_ = big_island_speed_scale_;
      gyro_scale_ = exchange_gyro_scale_;
      break;
    default:
      speed_change_scale_ = normal_speed_scale_;
      gyro_scale_ = normal_gyro_scale_;
      break;
  }
}

void Engineer2Manual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::checkKeyboard(dbus_data);
  ctrl_a_event_.update(dbus_data->key_ctrl & dbus_data->key_a);
  ctrl_b_event_.update(dbus_data->key_ctrl & dbus_data->key_b);
  ctrl_c_event_.update(dbus_data->key_ctrl & dbus_data->key_c);
  ctrl_d_event_.update(dbus_data->key_ctrl & dbus_data->key_d);
  ctrl_e_event_.update(dbus_data->key_ctrl & dbus_data->key_e);
  ctrl_f_event_.update(dbus_data->key_ctrl & dbus_data->key_f);
  ctrl_g_event_.update(dbus_data->key_ctrl & dbus_data->key_g);
  ctrl_q_event_.update(dbus_data->key_ctrl & dbus_data->key_q);
  ctrl_r_event_.update(dbus_data->key_ctrl & dbus_data->key_r);
  ctrl_s_event_.update(dbus_data->key_ctrl & dbus_data->key_s);
  ctrl_v_event_.update(dbus_data->key_ctrl & dbus_data->key_v);
  ctrl_w_event_.update(dbus_data->key_ctrl & dbus_data->key_w);
  ctrl_x_event_.update(dbus_data->key_ctrl & dbus_data->key_x);
  ctrl_z_event_.update(dbus_data->key_ctrl & dbus_data->key_z);

  b_event_.update(dbus_data->key_b & !dbus_data->key_ctrl & !dbus_data->key_shift);
  c_event_.update(dbus_data->key_c & !dbus_data->key_ctrl & !dbus_data->key_shift);
  e_event_.update(dbus_data->key_e & !dbus_data->key_ctrl & !dbus_data->key_shift);
  f_event_.update(dbus_data->key_f & !dbus_data->key_ctrl & !dbus_data->key_shift);
  g_event_.update(dbus_data->key_g & !dbus_data->key_ctrl & !dbus_data->key_shift);
  q_event_.update(dbus_data->key_q & !dbus_data->key_ctrl & !dbus_data->key_shift);
  r_event_.update(dbus_data->key_r & !dbus_data->key_ctrl & !dbus_data->key_shift);
  v_event_.update(dbus_data->key_v & !dbus_data->key_ctrl & !dbus_data->key_shift);
  x_event_.update(dbus_data->key_x & !dbus_data->key_ctrl & !dbus_data->key_shift);
  z_event_.update(dbus_data->key_z & !dbus_data->key_ctrl & !dbus_data->key_shift);

  shift_event_.update(dbus_data->key_shift & !dbus_data->key_ctrl);
  shift_b_event_.update(dbus_data->key_shift & dbus_data->key_b);
  shift_c_event_.update(dbus_data->key_shift & dbus_data->key_c);
  shift_e_event_.update(dbus_data->key_shift & dbus_data->key_e);
  shift_f_event_.update(dbus_data->key_shift & dbus_data->key_f);
  shift_g_event_.update(dbus_data->key_shift & dbus_data->key_g);
  shift_q_event_.update(dbus_data->key_shift & dbus_data->key_q);
  shift_r_event_.update(dbus_data->key_shift & dbus_data->key_r);
  shift_v_event_.update(dbus_data->key_shift & dbus_data->key_v);
  shift_x_event_.update(dbus_data->key_shift & dbus_data->key_x);
  shift_z_event_.update(dbus_data->key_shift & dbus_data->key_z);

  mouse_left_event_.update(dbus_data->p_l);
  mouse_right_event_.update(dbus_data->p_r);

}

void Engineer2Manual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::updateRc(dbus_data);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  chassis_cmd_sender_->getMsg()->command_source_frame = "base_link";
  vel_cmd_sender_->setAngularZVel(dbus_data->wheel);
  vel_cmd_sender_->setLinearXVel(dbus_data->ch_r_y);
  vel_cmd_sender_->setLinearYVel(-dbus_data->ch_r_x);
  left_switch_up_event_.update(dbus_data->s_l == rm_msgs::DbusData::UP);
  left_switch_down_event_.update(dbus_data->s_l == rm_msgs::DbusData::DOWN);
}

void Engineer2Manual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  checkKeyboard(dbus_data);
  left_switch_up_event_.update(dbus_data->s_l == rm_msgs::DbusData::UP);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  if (control_mode_ == CUSTOM)
    chassis_cmd_sender_->getMsg()->command_source_frame = "exchange_orientation";
  // else
  //   chassis_cmd_sender_->getMsg()->command_source_frame = "base_link";
  if (control_mode_ == JOINT)
    vel_cmd_sender_->setAngularZVel(-dbus_data->m_x * gimbal_scale_);
}

void Engineer2Manual::updateServo(const rm_msgs::VTReceiverControlData::ConstPtr& vt_control_data)
{
  // joint_jog_cmd_.header.stamp = ros::Time::now();
  // joint_jog_cmd_.header.frame_id = "base_link";
  // joint_jog_cmd_.joint_names = joint_servo_joint_names_;
  // joint_jog_cmd_.velocities.assign(joint_servo_joint_names_.size(), 0.0);
  // joint_jog_cmd_.displacements.clear();
  // joint_jog_cmd_.duration = 0.0;
  servo_cmd_.header.stamp = ros::Time::now();
  servo_cmd_.header.frame_id = "link7";
  servo_cmd_.twist.linear.x = vt_control_data->joystick_l_x * -0.2;
  servo_cmd_.twist.linear.y = vt_control_data->joystick_l_y * -0.2;
  servo_cmd_.twist.linear.z = linear_z_;
  servo_cmd_.twist.angular.x = vt_control_data->joystick_r_y * -1.0;
  servo_cmd_.twist.angular.y = vt_control_data->joystick_r_x * -1.0;
  servo_cmd_.twist.angular.z = vt_control_data->wheel * -1.0;
  // if (joint_jog_cmd_.velocities.size() > 0) joint_jog_cmd_.velocities[0] = joint1_scale_ * joint_servo_max_vel_[0]; // joint1
  // if (joint_jog_cmd_.velocities.size() > 1) joint_jog_cmd_.velocities[1] = -vt_control_data->joystick_l_x * joint_servo_max_vel_[1]; // joint2
  // if (joint_jog_cmd_.velocities.size() > 2) joint_jog_cmd_.velocities[2] = vt_control_data->joystick_l_y * joint_servo_max_vel_[2]; // joint3
  // if (joint_jog_cmd_.velocities.size() > 3) joint_jog_cmd_.velocities[3] = -vt_control_data->joystick_r_y * joint_servo_max_vel_[3]; // joint4
  // if (joint_jog_cmd_.velocities.size() > 4) joint_jog_cmd_.velocities[4] = -vt_control_data->joystick_r_x * joint_servo_max_vel_[4]; // joint5
  // if (joint_jog_cmd_.velocities.size() > 5) joint_jog_cmd_.velocities[5] = vt_control_data->wheel * joint_servo_max_vel_[5]; //joint6
  // if (joint_jog_cmd_.velocities.size() > 6) joint_jog_cmd_.velocities[6] = joint7_scale_ * joint_servo_max_vel_[6]; //joint7
}

void Engineer2Manual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
  // if (control_mode_ == SERVO)
  //   updateServo(data);
}



void Engineer2Manual::gpioStateCallback(const rm_msgs::GpioData::ConstPtr& data)
{
  gripper_ui_.state = { data->gpio_state.begin(), data->gpio_state.end() - 2 };
  engineer_ui_.main_gripper_state = data->gpio_state[0];
  engineer_ui_.small_gripper_state = data->gpio_state[2];
  // main_gripper_on_ = data->gpio_state[0];
  small_gripper_on_ = data->gpio_state[2];
}

void Engineer2Manual::sendCommand(const ros::Time& time)
{
  if (operating_mode_ == MANUAL)
  {
    chassis_cmd_sender_->sendChassisCommand(time, false);
    vel_cmd_sender_->sendCommand(time);
  }
  else if (operating_mode_ == MIDDLEWARE && chassis_cmd_sender_->getMsg()->command_source_frame != "base_link")
  {
    chassis_cmd_sender_->getMsg()->command_source_frame = "base_link";
    chassis_cmd_sender_->sendChassisCommand(time, false);
  }
  if (control_mode_ == SERVO)
  {
    changeSpeedMode(EXCHANGE);
    servo_cmd_pub_.publish(servo_cmd_);
    // joint_jog_pub_.publish(joint_jog_cmd_);
    // servo_command_sender_->sendCommand(time);
    if (gimbal_mode_ == RATE)
      gimbal_cmd_sender_->sendCommand(time);
  }
  if (control_mode_ == CUSTOM)
  {
    changeSpeedMode(EXCHANGE);
    if (gimbal_mode_ == RATE)
      gimbal_cmd_sender_->sendCommand(time);
  }
}

void Engineer2Manual::runStepQueue(const std::string& step_queue_name)
{
  rm_msgs::EngineerGoal goal;
  goal.step_queue_name = step_queue_name;
  if (action_client_.isServerConnected())
  {
    if (operating_mode_ == MANUAL)
      action_client_.sendGoal(goal, boost::bind(&Engineer2Manual::actionDoneCallback, this, _1, _2),
                              boost::bind(&Engineer2Manual::actionActiveCallback, this),
                              boost::bind(&Engineer2Manual::actionFeedbackCallback, this, _1));
    operating_mode_ = MIDDLEWARE;
  }
  else
    ROS_ERROR("Can not connect to middleware");
}

void Engineer2Manual::actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr& feedback)
{
}

void Engineer2Manual::actionDoneCallback(const actionlib::SimpleClientGoalState& state,
                                         const rm_msgs::EngineerResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result: %i", result->finish);
  ROS_INFO("Done %s", (prefix_ + root_).c_str());
  mouse_left_pressed_ = true;
  mouse_right_pressed_ = true;
  vt_trigger_pressed_ = true;
  is_in_synch_ = SynchStatus::WITHOUT_SYNCH;
  ROS_INFO("%i", result->finish);
  operating_mode_ = MANUAL;
  if (root_ == "HOME" )
  {
    engineer_ui_.step_queue_name = "";
    initMode();
    changeSpeedMode(NORMAL);
    is_first_stone_ = true;
    is_island_ = false;
    big_island_right_ = false;
    need_transfer_ = false;
  }
  if (prefix_ == "GET_STORED_" || root_ == "READY_ASSEMBLE")
  {
    enterCustom();
  }
  if (prefix_ == "GET_UNITS_")
  {
    if (prefix_ + root_ != "GET_UNITS_UP000" && prefix_ + root_ != "GET_UNITS_DOWN000")
    {
      chassis_cmd_sender_->getMsg()->command_source_frame = "exchange_orientation";
      changeSpeedMode(LOW);
    }
    else
    {
      chassis_cmd_sender_->getMsg()->command_source_frame = "base_link";
      initMode();
    }
  }
  if (prefix_ + root_ == "LIFTER_UP0" || prefix_ + root_ == "LIFTER_DOWN0")
  {
    initMode();
  }
}

void Engineer2Manual::enterServo()
{
  control_mode_ = SERVO;
  gimbal_mode_ = DIRECT;
  changeSpeedMode(EXCHANGE);
  servo_reset_caller_->callService();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  action_client_.cancelAllGoals();
  chassis_cmd_sender_->getMsg()->command_source_frame = "exchange_orientation";
  engineer_ui_.control_mode = "SERVO";
}

void Engineer2Manual::initMode()
{
  control_mode_ = JOINT;
  gimbal_mode_ = DIRECT;
  changeSpeedMode(NORMAL);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  chassis_cmd_sender_->getMsg()->command_source_frame = "base_link";
  engineer_ui_.control_mode = "NORMAL";
}

void Engineer2Manual::remoteControlTurnOff()
{
  ManualBase::remoteControlTurnOff();
  action_client_.cancelAllGoals();
}

void Engineer2Manual::gimbalOutputOn()
{
  ChassisGimbalManual::gimbalOutputOn();
}

void Engineer2Manual::chassisOutputOn()
{
  if (operating_mode_ == MIDDLEWARE)
    action_client_.cancelAllGoals();
}

//-------------------controller input-------------------
void Engineer2Manual::rightSwitchUpRise()
{
  ChassisGimbalManual::rightSwitchUpRise();
  gimbal_mode_ = DIRECT;
  control_mode_ = JOINT;
  initMode();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}
void Engineer2Manual::rightSwitchMidRise()
{
  ChassisGimbalManual::rightSwitchMidRise();
  control_mode_ = JOINT;
  gimbal_mode_ = RATE;
  gimbal_cmd_sender_->setZero();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}
void Engineer2Manual::rightSwitchDownRise()
{
   ChassisGimbalManual::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
//  control_mode_ = SERVO;
  control_mode_ = CUSTOM;
  gimbal_mode_ = DIRECT;
//  servo_reset_caller_->callService();
  action_client_.cancelAllGoals();
//  ROS_INFO_STREAM("servo_mode");
  ROS_INFO_STREAM("custom_controller_mode");
}

void Engineer2Manual::leftSwitchUpRise()
{
  prefix_ = "";
  root_ = "CALIBRATION";
  // runStepQueue("CALIBRATION");
  calibration_gather_->reset();
  // engineer_ui_.control_mode = "NORMAL";
  ROS_INFO_STREAM("START CALIBRATE");
}
void Engineer2Manual::leftSwitchUpFall()
{
  prefix_ = "";
  root_ = "HOME";
  runStepQueue("HOME");
}

void Engineer2Manual::leftSwitchDownRise()
{
  std_srvs::Trigger srv;
  if (gripper_state_caller_.exists())
    gripper_state_caller_.call(srv);
  else
    return;
  prefix_ = "GRIPPER_";
  if (!srv.response.success)
  {
    root_ = "CLOSE";
    runStepQueue(prefix_ + root_);
  }
  else
  {
    root_ = "OPEN";
    runStepQueue(prefix_ + root_);
  }
}
void Engineer2Manual::leftSwitchDownFall()
{
  // runStepQueue("MIDDLE_PITCH_UP");
}

//--------------------- keyboard input ------------------------
// mouse input
void Engineer2Manual::mouseLeftRelease()
{
  if (mouse_left_pressed_)
  {
    root_ += "0";
    mouse_left_pressed_ = false;
    runStepQueue(prefix_ + root_);
    ROS_INFO("Finished %s", (prefix_ + root_).c_str());
  }
}

void Engineer2Manual::mouseRightRelease()
{
  // runStepQueue(prefix_ + root_);
  // ROS_INFO("Finished %s", (prefix_ + root_).c_str());
}

//   void Engineer2Manual::wPressing()
// {
//   double final_x_scale = x_scale_ * speed_change_scale_;
//   if (is_island_)
//     vel_cmd_sender_->setLinearXVel(final_x_scale * 2.0);
//   else
//     vel_cmd_sender_->setLinearXVel(final_x_scale);
// }
//
//   void Engineer2Manual::aPressing()
// {
//   double final_y_scale = y_scale_ * speed_change_scale_;
//   if (big_island_right_)
//     vel_cmd_sender_->setLinearYVel(final_y_scale * 4.0);
//   else
//     vel_cmd_sender_->setLinearYVel(final_y_scale);
// }
//
//   void Engineer2Manual::sPressing()
// {
//   double final_x_scale = x_scale_ * speed_change_scale_;
//   if (is_island_)
//     vel_cmd_sender_->setLinearXVel(final_x_scale * 2.0);
//   else
//     vel_cmd_sender_->setLinearXVel(final_x_scale);
// }
//
//   void Engineer2Manual::dPressing()
// {
//   double final_y_scale = y_scale_ * speed_change_scale_;
//   if (big_island_right_)
//     vel_cmd_sender_->setLinearYVel(final_y_scale * 4.0);
//   else
//     vel_cmd_sender_->setLinearYVel(final_y_scale);
// }

void Engineer2Manual::bPressing()
{
}

void Engineer2Manual::bRelease()
{
}
void Engineer2Manual::cPressing()
{
  // angular_z_scale_ = -0.8;
}
void Engineer2Manual::cRelease()
{
  // angular_z_scale_ = 0.;
}
void Engineer2Manual::ePressing()
{
  // if (control_mode_ == SERVO || control_mode_ == CUSTOM)
  //   vel_cmd_sender_->setAngularZVel(-gyro_scale_);
}
void Engineer2Manual::eRelease()
{
  // if (control_mode_ == SERVO || control_mode_ == CUSTOM)
  //   vel_cmd_sender_->setAngularZVel(0.);
}
void Engineer2Manual::fPress()
{
}
void Engineer2Manual::fRelease()
{
}
void Engineer2Manual::gPress()
{
}
void Engineer2Manual::gRelease()
{
}
void Engineer2Manual::qPressing()
{
  // if (control_mode_ == SERVO || control_mode_ == CUSTOM)
  //   vel_cmd_sender_->setAngularZVel(gyro_scale_);
}
void Engineer2Manual::qRelease()
{
  // if (control_mode_ == SERVO || control_mode_ == CUSTOM)
  //   vel_cmd_sender_->setAngularZVel(0.);
}
void Engineer2Manual::rPress()
{
  if (prefix_ + root_ == "STORE_UNIT_BACK") {
    root_ = "BACK_R";
    runStepQueue(prefix_ + root_);
    ROS_INFO("%s", (prefix_ + root_).c_str());
  }
  else if (prefix_ + root_ == "STORE_UNIT_FRONT") {
    root_ = "FRONT_R";
    runStepQueue(prefix_ + root_);
    ROS_INFO("%s", (prefix_ + root_).c_str());
  }
  else if (prefix_ + root_ == "STORE_UNIT_BACK_R") {
    root_ = "BACK";
    runStepQueue(prefix_ + root_);
    ROS_INFO("%s", (prefix_ + root_).c_str());
  }
  else if (prefix_ + root_ == "STORE_UNIT_FRONT_R") {
    root_ = "FRONT";
    runStepQueue(prefix_ + root_);
    ROS_INFO("%s", (prefix_ + root_).c_str());
  }
}

void Engineer2Manual::rRelease()
{
}

void Engineer2Manual::vPressing()
{
}
void Engineer2Manual::vRelease()
{
}
void Engineer2Manual::xPress()
{
  // if (control_mode_ == SERVO || control_mode_ == CUSTOM)
  // {
  //   prefix_ = "";
  //   root_ = "";
  //   switch (gimbal_direction_)
  //   {
  //     case 0:
  //       root_ = "GIMBAL_EE";
  //       gimbal_direction_ = 1;
  //       break;
  //     case 1:
  //       root_ = "GIMBAL_F";
  //       gimbal_direction_ = 0;
  //       break;
  //     default:
  //       gimbal_direction_ = 0;
  //       break;
  //   }
  //   runStepQueue(prefix_ + root_);
  // }
  // else
  // {
  //   switch (gimbal_direction_)
  //   {
  //     case 0:
  //       runStepQueue("GIMBAL_R");
  //       gimbal_direction_ = 1;
  //       break;
  //     case 1:
  //       runStepQueue("GIMBAL_B");
  //       gimbal_direction_ = 2;
  //       break;
  //     case 2:
  //       runStepQueue("GIMBAL_F");
  //       gimbal_direction_ = 0;
  //       break;
  //     default:
  //       gimbal_direction_ = 0;
  //       break;
  //   }
  // }
}
void Engineer2Manual::zPressing()
{
  // angular_z_scale_ = 0.8;
}
void Engineer2Manual::zRelease()
{
  // angular_z_scale_ = 0.;
}
void Engineer2Manual::zPress()
{
  // if (chassis_locked_)
  //   chassis_locked_ = false;
  // else
  //   chassis_locked_ = true;
}

//---------------------  CTRL  ---------------------
void Engineer2Manual::ctrlAPress()
{
  initMode();
  prefix_ = "GET_STORED_";
  root_ = "BACK";
  runStepQueue(prefix_ + root_);
  // changeSpeedMode(LOW);
  ROS_INFO("%s", (prefix_ + root_).c_str());
  // small_arm_stone_ = SILVER;
  // engineer_ui_.step_queue_name = "SMALL_ISLAND";
}
void Engineer2Manual::ctrlBPress()
{
  initMode();
  prefix_ = "";
  root_ = "HOME";
  gimbal_direction_ = 0;
  runStepQueue(prefix_ + root_);
  changeSpeedMode(NORMAL);
}
void Engineer2Manual::ctrlBPressing()
{
}
void Engineer2Manual::ctrlBRelease()
{
}
void Engineer2Manual::ctrlCPress()
{
  action_client_.cancelAllGoals();
  changeSpeedMode(NORMAL);
  initMode();
  ROS_INFO("cancel all goal");
}
void Engineer2Manual::ctrlDPress()
{
  initMode();
  prefix_ = "GET_STORED_";
  root_ = "FRONT";
  // // had_side_gold_ = true;
  // // had_ground_stone_ = false;
  // changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
  // small_arm_stone_ = GOLD;
  // engineer_ui_.step_queue_name = "BIG_ISLAND";
}
void Engineer2Manual::ctrlEPress()
{
  initMode();
  prefix_ = "STORE_UNIT_";
  root_ = "FRONT";
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}
void Engineer2Manual::ctrlFPress()
{
  // initMode();
  // enterCustom();
  // chassis_locked_ = true;
  // if (need_transfer_)
  //   prefix_ = "READY_TRANSFER_";
  // else
  //   prefix_ = "READY_";
  // root_ = "EXCHANGE";
  prefix_ = "";
  root_ = "READY_ASSEMBLE";
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}
void Engineer2Manual::ctrlGPress()
{
  std_srvs::Trigger srv;
  if (gripper_state_caller_.exists())
    gripper_state_caller_.call(srv);
  else
    return;
  prefix_ = "GRIPPER_";
  if (!srv.response.success)
  {
    root_ = "CLOSE";
    runStepQueue(prefix_ + root_);
  }
  else
  {
    root_ = "OPEN";
    runStepQueue(prefix_ + root_);
  }
  ROS_INFO("%s", (prefix_ + root_).c_str());
}
void Engineer2Manual::ctrlQPress()
{
  initMode();
  prefix_ = "STORE_UNIT_";
  root_ = "BACK";
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}
void Engineer2Manual::ctrlRPress()
{
  initMode();
  prefix_ = "LIFTER_";
  root_ = "INIT";
  runStepQueue(prefix_ + root_);
  control_mode_ = JOINT;
  calibration_gather_->reset();
  ROS_INFO_STREAM("START CALIBRATE");
  changeSpeedMode(NORMAL);
}
void Engineer2Manual::ctrlSPress()
{
  prefix_ = "GET_UNITS_";
  root_ = "DOWN";
  changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}
void Engineer2Manual::ctrlVPress()
{
}
void Engineer2Manual::ctrlVRelease()
{
}
void Engineer2Manual::ctrlWPress()
{
  prefix_ = "GET_UNITS_";
  root_ = "UP";
  changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}
void Engineer2Manual::ctrlXPress()
{
  prefix_ = "LIFTER_";
  root_ = "UP";
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}
void Engineer2Manual::ctrlZPress()
{
  prefix_ = "LIFTER_";
  root_ = "DOWN";
  changeSpeedMode(BIG_ISLAND_SPEED);
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

//---------------  SHIFT  --------------------------

void Engineer2Manual::shiftPressing()
{
  changeSpeedMode(FAST);
}
void Engineer2Manual::shiftRelease()
{
  changeSpeedMode(NORMAL);
}
void Engineer2Manual::shiftBPress()
{
}
void Engineer2Manual::shiftBRelease()
{
}
void Engineer2Manual::shiftCPress()
{
}
void Engineer2Manual::shiftEPress()
{
}
void Engineer2Manual::shiftFPress()
{
}
void Engineer2Manual::shiftGPress()
{
}
void Engineer2Manual::shiftQPress()
{
}
void Engineer2Manual::shiftRPress()
{
}
void Engineer2Manual::shiftRRelease()
{
}
void Engineer2Manual::shiftVPress()
{
  // prefix_ = "";
  // if (main_gripper_on_)
  // {
  //   root_ = "CM";
  // }
  // else
  // {
  //   root_ = "OM";
  // }
  // runStepQueue(root_);
}
void Engineer2Manual::shiftVRelease()
{
}
void Engineer2Manual::shiftXPress()
{
}
void Engineer2Manual::shiftZPress()
{
  // prefix_ = "";
  // root_ = "CG";
  // runStepQueue(prefix_ + root_);
  // ROS_INFO("%s", (prefix_ + root_).c_str());
}
void Engineer2Manual::shiftZRelease()
{
}

//custom controller
void Engineer2Manual::enterCustom()
{
  custom_last_ = ros::Time::now();
  control_mode_ = CUSTOM;
  gimbal_mode_ = DIRECT;
  changeSpeedMode(EXCHANGE);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  action_client_.cancelAllGoals();
  chassis_cmd_sender_->getMsg()->command_source_frame = "exchange_orientation";
  engineer_ui_.control_mode = "CUSTOM";
}
void Engineer2Manual::distSensorCallback(const std_msgs::Float64::ConstPtr& angle_data)
{
  yaw_lock_error_ = angle_data->data;
}
void Engineer2Manual::customControllerCallback(const rm_msgs::CustomControllerData::ConstPtr& custom_data)
{
  if (control_mode_ == CUSTOM)
    updateCustomController(custom_data);
}
void Engineer2Manual::updateCustomController(const rm_msgs::CustomControllerData::ConstPtr& custom_data)
{
  checkCustomButton(custom_data);
  double custom_joint_state_[6], joint1_state_ = joint_state_.position[1],
  joystick_l_x = abs(custom_data->joystick_l_x_data - custom_joystick_offset_[0]) > custom_joystick_dead_zone_ ? (custom_data->joystick_l_x_data - custom_joystick_offset_[0]) * 1.0 / custom_joystick_offset_[0] : 0.0,
  joystick_l_y = abs(custom_data->joystick_l_y_data - custom_joystick_offset_[1]) > custom_joystick_dead_zone_ ? (custom_data->joystick_l_y_data - custom_joystick_offset_[1]) * 1.0 / custom_joystick_offset_[1] : 0.0,
  joystick_r_x = abs(custom_data->joystick_r_x_data - custom_joystick_offset_[2]) > custom_joystick_dead_zone_ ? (custom_data->joystick_r_x_data - custom_joystick_offset_[2]) * 1.0 / custom_joystick_offset_[2] : 0.0,
  joystick_r_y = abs(custom_data->joystick_r_y_data - custom_joystick_offset_[3]) > custom_joystick_dead_zone_ ? (custom_data->joystick_r_y_data - custom_joystick_offset_[3]) * 1.0 / custom_joystick_offset_[3] : 0.0;
  trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
  trajectory_msgs::JointTrajectory joint_trajectory;
  for( size_t i = 0; i < 6; i++ )
  {
    double joint_data = custom_data->encoder_data[i] + custom_data_offset_[i];
    if( joint_data >= 6.28 )
      joint_data -= 6.28;
    if( joint_data < 0)
      joint_data += 6.28;
    joint_data *= custom2joint_orientation_[i];
    joint_data += custom2joint_offset_[i];
    custom_joint_state_[i] = joint_data;
  }
  double joint4_limit = -0.21 - custom_joint_state_[1];
  custom_joint_state_[2] = custom_joint_state_[2] >= joint4_limit ? custom_joint_state_[2] : joint4_limit;
  joint_trajectory.header.seq = custom_seq_;
  joint_trajectory.joint_names = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7" };
  joint_trajectory_point.positions.push_back( joint1_state_ - joystick_l_x * joint1_speed_scale_ );

  bool near_synch = true;
  switch (is_in_synch_)
  {
    case SynchStatus::WITHOUT_SYNCH:
      start_synch_time_ = ros::Time::now();
      is_in_synch_ = SynchStatus::SYNCHRONIZING;
    case SynchStatus::SYNCHRONIZING:
      joint_trajectory.header.stamp = start_synch_time_;
      joint_trajectory_point.time_from_start = ros::Duration(1.0);
      near_synch &= abs(custom_joint_state_[0] - joint_state_.position[2]) < 0.314;
      near_synch &= abs(custom_joint_state_[1] - joint_state_.position[3]) < 0.314;
      near_synch &= abs(custom_joint_state_[2] - joint_state_.position[4]) < 0.314;
      near_synch &= abs(custom_joint_state_[3] - joint_state_.position[5]) < 0.314;
      near_synch &= abs(custom_joint_state_[4] - joint_state_.position[6]) < 0.314;
      near_synch &= abs(custom_joint_state_[5] - joint_state_.position[7]) < 0.314;
      if (ros::Time::now() - start_synch_time_ > ros::Duration(1.0) || near_synch)
      {
        is_in_synch_ = SynchStatus::IN_SYNCH;
      }
      break;
    case SynchStatus::IN_SYNCH:
      joint_trajectory.header.stamp = ros::Time::now();
      joint_trajectory_point.time_from_start = ros::Duration(0.2);
      break;
  }

  for( double i : custom_joint_state_ )
  {
    joint_trajectory_point.positions.push_back( i );
  }
  joint_trajectory.points.push_back( joint_trajectory_point );
  trajectory_cmd_pub_.publish( joint_trajectory );
  custom_seq_++;
  if( custom_seq_ > 100000 )
    custom_seq_ = 1;
  double angular_z_cmd;
  if (chassis_locked_)
    angular_z_cmd = -joystick_l_y == 0.0 ? yaw_lock_pid_.computeCommand(yaw_lock_error_, ros::Time::now() - custom_last_) : -joystick_l_y;
  else
    angular_z_cmd =  -joystick_l_y;
  if (abs(angular_z_cmd)  > 2.0)
    angular_z_cmd = angular_z_cmd > 0 ? 2.0 : -2.0;
  vel_cmd_sender_->setAngularZVel(angular_z_cmd);
  vel_cmd_sender_->setLinearXVel(-joystick_r_x * 0.7);
  vel_cmd_sender_->setLinearYVel(-joystick_r_y * 0.7);
  custom_last_ = ros::Time::now();
}
void Engineer2Manual::checkCustomButton(const rm_msgs::CustomControllerData::ConstPtr& custom_data)
{
  custom_button1_event_.update(custom_data->button_data[0]);
  custom_button2_event_.update(custom_data->button_data[1]);
  custom_button3_event_.update(custom_data->button_data[2]);
  custom_button4_event_.update(custom_data->button_data[3]);
}

void Engineer2Manual::customButton1Press()
{
  // ROS_INFO("button1 pressed");
}
void Engineer2Manual::customButton2Press()
{
  // if (control_mode_ == SERVO || control_mode_ == CUSTOM)
  // {
  //   switch (gimbal_direction_)
  //   {
  //     case 0:
  //       runStepQueue("GIMBAL_EE");
  //       gimbal_direction_ = 1;
  //       break;
  //     case 1:
  //       runStepQueue("GIMBAL_R");
  //       gimbal_direction_ = 0;
  //       break;
  //     default:
  //       gimbal_direction_ = 0;
  //       break;
  //   }
  // }
}
void Engineer2Manual::customButton3Press()
{
  std_srvs::Trigger srv;
  if (gripper_state_caller_.exists())
    gripper_state_caller_.call(srv);
  else
    return;
  prefix_ = "GRIPPER_";
  root_ = "CLOSE";
  runStepQueue(prefix_ + root_);
}
void Engineer2Manual::customButton4Press()
{
  std_srvs::Trigger srv;
  if (gripper_state_caller_.exists())
    gripper_state_caller_.call(srv);
  else
    return;
  prefix_ = "GRIPPER_";
  root_ = "OPEN";
  runStepQueue(prefix_ + root_);
  // initMode();
  // if (is_first_stone_)
  // {
  //   prefix_ = "GET_SMALL_ARM_STONE_FROM_";
  //   switch (small_arm_stone_)
  //   {
  //     case EMPTY:
  //       ROS_INFO_STREAM("SMALL_ARM_HAS_NO_STONE");
  //       prefix_ = "";
  //       root_ = "";
  //       break;
  //     case GOLD:
  //       root_ = "SIDE";
  //       break;
  //     case SILVER:
  //       root_ = "UP";
  //       break;
  //   }
  //   runStepQueue(prefix_ + root_);
  //   is_first_stone_ = false;
  // }
  // else
  // {
  //   prefix_ = "";
  //   root_ = "HOME_TURN_BACK";
  //   gimbal_direction_ = 0;
  //   runStepQueue(prefix_ + root_);
  //   changeSpeedMode(NORMAL);
  // }
}

//vt control
void Engineer2Manual::vtControlCallback(const rm_msgs::VTReceiverControlData::ConstPtr &vt_control_data)
{
  // if (state_ == PC)
  //   updateVTControl(vt_control_data);
}
void Engineer2Manual::updateVTControl(const rm_msgs::VTReceiverControlData::ConstPtr &vt_control_data)
{
  vt_mode_c_event_.update(vt_control_data->mode_switch == rm_msgs::VTReceiverControlData::MODE_C);
  vt_mode_n_event_.update(vt_control_data->mode_switch == rm_msgs::VTReceiverControlData::MODE_N);
  vt_mode_s_event_.update(vt_control_data->mode_switch == rm_msgs::VTReceiverControlData::MODE_S);
  if (control_mode_ == SERVO)
  {
    vt_custom_button_l_event_.update(vt_control_data->custom_button_l);
    vt_custom_button_r_event_.update(vt_control_data->custom_button_r);
    vt_pause_button_event_.update(vt_control_data->pause_button);
    vt_trigger_event_.update(vt_control_data->trigger);
    updateServo(vt_control_data);
  }
}

void Engineer2Manual::vtModeN()
{
  // if (control_mode_ == JOINT)
  //   control_mode_ = VT_CONTROL;
}
void Engineer2Manual::vtModeC()
{
  // if (control_mode_ == VT_CONTROL)
  //   control_mode_ = JOINT;
}
void Engineer2Manual::vtModeSRise()
{
  enterServo();
  ROS_INFO("enter servo mode");
}
void Engineer2Manual::vtModeSFall()
{
  initMode();
}
void Engineer2Manual::vtCustomButtonLPress()
{
  linear_z_ = 0.1;
}
void Engineer2Manual::vtCustomButtonLRelease()
{
  linear_z_ = 0.0;
}
void Engineer2Manual::vtCustomButtonRPress()
{
  linear_z_ = -0.1;
}
void Engineer2Manual::vtCustomButtonRRelease()
{
  linear_z_ = 0.0;
}

// void Engineer2Manual::vtCustomButtonLPress()
// {
//   joint7_scale_ = 1.0;
// }
// void Engineer2Manual::vtCustomButtonLRelease()
// {
//   joint7_scale_ = 0.0;
// }
// void Engineer2Manual::vtCustomButtonRPress()
// {
//   joint7_scale_ = -1.0;
// }
// void Engineer2Manual::vtCustomButtonRRelease()
// {
//   joint7_scale_ = 0.0;
// }
// void Engineer2Manual::vtPauseButtonPress()
// {
//   joint1_scale_ = -1.0;
// }
// void Engineer2Manual::vtPauseButtonRelease()
// {
//   joint1_scale_ = 0.0;
// }
// void Engineer2Manual::vtTriggerPress()
// {
//   joint1_scale_ = 1.0;
// }
// void Engineer2Manual::vtTriggerRelease()
// {
//   joint1_scale_ = 0.0;
// }
}  // namespace rm_manual
