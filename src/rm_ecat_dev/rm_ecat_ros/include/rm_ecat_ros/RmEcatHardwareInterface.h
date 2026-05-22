//
// Created by qiayuan on 23-4-16.
//
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <urdf/model.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/robot_hw.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <rm_common/hardware_interface/actuator_extra_interface.h>
#include <rm_common/hardware_interface/gpio_interface.h>
#include <rm_common/hardware_interface/rm_imu_sensor_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>

#include <joint_limits_interface/joint_limits_interface.h>
#include <transmission_interface/transmission_interface_loader.h>

#include "rm_ecat_ros/RmMitSlaveManagerRos.h"
#include "rm_ecat_ros/RmStandardSlaveManagerRos.h"
#include "rm_msgs/BusState.h"
#include "rm_msgs/GpioData.h"

namespace rm_ecat {
struct ActData {
  double pos, vel, effort;
  double commandUnlimited, command;
  double offset;
  bool halted, needCalibration, calibrated, calibrationReading;
};

struct ImuData {
  ros::Time stamp;
  double orientation[4], angularVel[3], linearAccel[3];
  double orientationCovariance[3], angularVelocityCovariance[3], linearAccelerationCovariance[3];
};

class RmEcatHardwareInterface : public hardware_interface::RobotHW {
 public:
  RmEcatHardwareInterface() = default;

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  bool isRunning();

 protected:
  bool updateWorkerCb(const any_worker::WorkerEvent& /*event*/);
  bool controlWorkerCb(const any_worker::WorkerEvent& /*event*/);
  bool publishWorkerCb(const any_worker::WorkerEvent& /*event*/);
  void handleSignal(int signum);

  std::shared_ptr<ecat_manager::EcatBusManager> busManager_;
  std::set<std::string> busNames_;
  std::map<std::string, bool> busIsOk_;
  int busDiagDecimationCount_ = 0;
  std::shared_ptr<rm_ecat::standard::RmStandardSlaveManagerRos> rmStandardSlaveManager_;
  std::shared_ptr<rm_ecat::mit::RmMitSlaveManagerRos> rmMitSlaveManager_;

  std::shared_ptr<any_worker::Worker> updateWorker_;
  std::shared_ptr<any_worker::Worker> controlWorker_;
  std::shared_ptr<any_worker::Worker> publishWorker_;
  any_node::ThreadedPublisherPtr<rm_msgs::BusState> busStatesPublisher_;
  std::atomic<bool> busStatesMsgUpdated_;
  any_node::ThreadedPublisherPtr<rm_msgs::GpioData> rmGpioOutputsPublisher_;
  std::atomic<bool> rmGpioOutputsMsgUpdated_;
  any_node::ThreadedPublisherPtr<rm_msgs::GpioData> mitGpioOutputsPublisher_;
  std::atomic<bool> mitGpioOutputsMsgUpdated_;

  ////////// ros-control //////////
  bool loadUrdf(ros::NodeHandle& root_nh);
  void setupActuators();
  bool setupTransmission();
  bool setupJointLimit();
  void setupImus();
  void setupGpios();
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

  std::string urdf_string_;
  std::shared_ptr<urdf::Model> urdf_model_;

  std::shared_ptr<controller_manager::ControllerManager> controllerManager_;

  hardware_interface::ActuatorStateInterface act_state_interface_;
  rm_control::ActuatorExtraInterface act_extra_interface_;
  hardware_interface::EffortActuatorInterface effort_act_interface_;
  rm_control::RobotStateInterface robot_state_interface_;
  hardware_interface::ImuSensorInterface imu_sensor_interface_;
  rm_control::RmImuSensorInterface rm_imu_sensor_interface_;

  std::unique_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_{};
  transmission_interface::RobotTransmissions robot_transmissions_;
  transmission_interface::ActuatorToJointStateInterface* act_to_jnt_state_{};
  transmission_interface::JointToActuatorEffortInterface* jnt_to_act_effort_{};
  joint_limits_interface::EffortJointSaturationInterface effort_jnt_saturation_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface effort_jnt_soft_limits_interface_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_{};
  rm_control::GpioStateInterface gpio_state_interface_;
  rm_control::GpioCommandInterface gpio_command_interface_;

  std::list<ActData> act_data_list_;
  std::list<ImuData> imu_data_list_;
  std::list<bool> digital_input_list_;
  std::list<bool> digital_output_list_;
};

}  // namespace rm_ecat
