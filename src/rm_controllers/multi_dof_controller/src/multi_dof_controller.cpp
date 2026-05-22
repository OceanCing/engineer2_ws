//
// Created by lsy on 23-3-15.
//

#include "multi_dof_controller/multi_dof_controller.h"

#include <string>
#include <rm_common/ros_utilities.h>
#include <pluginlib/class_list_macros.hpp>

namespace multi_dof_controller
{
/**
 * 控制器初始化函数。
 * @param robot_hw 机器人硬件接口。
 * @param root_nh 根节点句柄。
 * @param controller_nh 控制器节点句柄。
 * @return 初始化成功返回true，失败返回false。
 */
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // 从控制器节点句柄中获取关节参数。
  XmlRpc::XmlRpcValue joints;
  controller_nh.getParam("joints", joints);

  // 获取位置容差和超时参数。
  position_tolerance_ = getParam(controller_nh, "position_tolerance", 0.01);
  time_out_ = getParam(controller_nh, "time_out", 1);

  // 断言joints参数类型为结构体。
  ROS_ASSERT(joints.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  // 获取努力关节接口。
  hardware_interface::EffortJointInterface* effort_joint_interface;
  effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();

  // 遍历关节参数，初始化关节控制器。
  for (const auto& joint : joints)
  {
    // 初始化关节对象，包括位置和速度控制器。
    Joint j{ .joint_name_ = joint.first,
             .ctrl_position_ = new effort_controllers::JointPositionController(),
             .ctrl_velocity_ = new effort_controllers::JointVelocityController() };

    // 为每个关节的position和velocity控制器创建节点句柄。
    ros::NodeHandle nh_position = ros::NodeHandle(controller_nh, "joints/" + joint.first + "/position");
    ros::NodeHandle nh_velocity = ros::NodeHandle(controller_nh, "joints/" + joint.first + "/velocity");

    // 初始化关节位置和速度控制器，失败则返回false。
    if (!j.ctrl_position_->init(effort_joint_interface, nh_position) ||
        !j.ctrl_velocity_->init(effort_joint_interface, nh_velocity))
      return false;

    // 将关节对象添加到关节列表。
    joints_.push_back(j);
  }

  // 从控制器节点句柄中获取运动参数。
  XmlRpc::XmlRpcValue motions;
  controller_nh.getParam("motions", motions);

  // 遍历运动参数，初始化运动对象。
  for (const auto& motion : motions)
  {
    // 初始化运动对象，包括最大速度。
    Motion m{ .motion_name_ = motion.first,
              .velocity_max_speed_ = xmlRpcGetDouble(motion.second["velocity_max_speed"]) };

    // 验证并获取运动的位置、速度和固定方向参数。
    for (int i = 0; i < (int)motion.second["position"].size(); ++i)
    {
      ROS_ASSERT(motion.second["position"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      ROS_ASSERT(motion.second["velocity"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      ROS_ASSERT(motion.second["fixed_direction"][i].getType() == XmlRpc::XmlRpcValue::TypeInt);

      m.position_.push_back(xmlRpcGetDouble(motion.second["position"][i]));
      m.velocity_.push_back(xmlRpcGetDouble(motion.second["velocity"][i]));
      m.fixed_direction_.push_back(xmlRpcGetDouble(motion.second["fixed_direction"][i]));
    }

    // 将运动对象添加到运动列表。
    motions_.push_back(m);
  }

  // 订阅命令消息。
  cmd_multi_dof_sub_ = controller_nh.subscribe("command", 1, &Controller::commandCB, this);

  // 初始化成功。
  return true;
}


void Controller::starting(const ros::Time& /*unused*/)
{
  state_ = rm_msgs::MultiDofCmd::VELOCITY;
  state_changed_ = true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  motion_group_.clear();
  motion_group_values_.clear();
  rm_msgs::MultiDofCmd cmd_multi_dof;
  cmd_multi_dof = *cmd_rt_buffer_.readFromRT();
  judgeMotionGroup(cmd_multi_dof);
  if (state_ != cmd_multi_dof.mode)
  {
    state_ = cmd_multi_dof.mode;
    state_changed_ = true;
  }
  switch (state_)
  {
    case rm_msgs::MultiDofCmd::VELOCITY:
      velocity(time, period);
      break;
    case rm_msgs::MultiDofCmd::POSITION:
      position(time, period);
      break;
  }
}

void Controller::velocity(const ros::Time& time, const ros::Duration& period)
{
  position_change_ = true;
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Multi_Dof] VELOCITY");
  }
  std::vector<double> results((double)joints_.size(), 0);
  for (int i = 0; i < (int)joints_.size(); ++i)
  {
    for (int j = 0; j < (int)motion_group_.size(); ++j)
    {
      for (int k = 0; k < (int)motions_.size(); ++k)
      {
        if (motions_[k].motion_name_ == motion_group_[j])
          results[i] += judgeInputDirection(motion_group_values_[j], motions_[k].fixed_direction_[i]) *
                        motions_[k].velocity_max_speed_ * motions_[k].velocity_[i];
      }
    }
  }
  for (int i = 0; i < (int)joints_.size(); ++i)
  {
    joints_[i].ctrl_velocity_->setCommand(results[i]);
    joints_[i].ctrl_velocity_->update(time, period);
  }
}
/**
 * @brief 控制器的位置更新函数
 *
 * 该函数负责根据当前时间及时间段更新控制器中每个关节的位置。它首先检查状态是否已变更，
 * 如果已变更，则记录相关信息并重置状态标志。然后，根据当前关节位置和设定的目标位置，
 * 更新每个关节的目标位置。最后，检查每个关节是否已到达目标位置，并相应地更新关节的目标位置。
 *
 * @param time 当前时间
 * @param period 时间段
 */
void Controller::position(const ros::Time& time, const ros::Duration& period)
{
  // 如果状态已变更，记录信息并重置状态标志
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Multi_Dof] POSITION");
  }

  // 如果位置需要变更，计算新的目标位置
  if (position_change_)
  {
    start_time_ = time;
    std::vector<double> current_positions((double)joints_.size(), 0);
    std::vector<double> results((double)joints_.size(), 0);
    targets_ = results;

    // 遍历每个关节，计算目标位置
    for (int i = 0; i < (int)joints_.size(); ++i)
    {
      current_positions[i] = (joints_[i].ctrl_position_->getPosition());
      for (int j = 0; j < (int)motion_group_.size(); ++j)
      {
        for (int k = 0; k < (int)motions_.size(); ++k)
        {
          if (motions_[k].motion_name_ == motion_group_[j])
          {
            results[i] += judgeInputDirection(motion_group_values_[j], motions_[k].fixed_direction_[i]) *
                          motions_[k].position_[i];
          }
        }
      }
      position_change_ = false;
      targets_[i] = results[i] + current_positions[i];
    }
  }

  // 计算已到达目标位置的关节数量
  double arrived_joint_num = 0;
  for (int i = 0; i < (int)joints_.size(); ++i)
  {
    // 如果关节已到达目标位置，更新目标位置为当前位置，并增加已到达关节的数量
    if (targets_[i] - position_tolerance_ <= joints_[i].ctrl_position_->getPosition() &&
        targets_[i] + position_tolerance_ >= joints_[i].ctrl_position_->getPosition())
    {
      arrived_joint_num++;
      targets_[i] = joints_[i].ctrl_position_->getPosition();
    }

    // 设置关节的目标位置并更新关节状态
    joints_[i].ctrl_position_->setCommand(targets_[i]);
    joints_[i].ctrl_position_->update(time, period);
  }

  // 如果所有关节均已到达目标位置，或超过设定的超时时间，标记位置需要变更
  if (arrived_joint_num == (int)joints_.size() || (time - start_time_).toSec() >= time_out_)
    position_change_ = true;
}


double Controller::judgeInputDirection(double value, bool fixed_direction)
{
  return fixed_direction ? abs(value) : value;
}
void Controller::judgeMotionGroup(rm_msgs::MultiDofCmd cmd_multi_dof)
{
  std::vector<std::string> motion_names = { "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z" };
  std::vector<double> motion_values = { cmd_multi_dof.linear.x,  cmd_multi_dof.linear.y,  cmd_multi_dof.linear.z,
                                        cmd_multi_dof.angular.x, cmd_multi_dof.angular.y, cmd_multi_dof.angular.z };
  for (int i = 0; i < (int)motion_names.size(); i++)
  {
    if (abs(motion_values[i]))
    {
      motion_group_.push_back(motion_names[i]);
      motion_group_values_.push_back(motion_values[i]);
    }
  }
}

void Controller::commandCB(const rm_msgs::MultiDofCmdPtr& msg)
{
  cmd_rt_buffer_.writeFromNonRT(*msg);
}
}  // namespace multi_dof_controller
PLUGINLIB_EXPORT_CLASS(multi_dof_controller::Controller, controller_interface::ControllerBase)
