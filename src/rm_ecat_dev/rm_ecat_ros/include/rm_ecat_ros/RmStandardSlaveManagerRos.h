//
// Created by qiayuan on 23-4-13.
//

#include <ros/ros.h>
#include <mutex>

#include <rm_ecat_manager/RmEcatStandardSlaveManager.h>
#include <rm_ecat_msgs/RmEcatStandardSlaveReadings.h>
#include <rm_msgs/EnableImuTrigger.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <any_node/ThreadedPublisher.hpp>

#include "rm_ecat_ros/ClearSlaveManagerRosBase.h"

namespace rm_ecat {
namespace standard {
class RmStandardSlaveManagerRos : public cleardrive_ros::ClearSlaveManagerRosBase<rm_ecat::standard::RmEcatStandardSlaveManager> {
 public:
  using ClearSlaveManagerRosBase<rm_ecat::standard::RmEcatStandardSlaveManager>::ClearSlaveManagerRosBase;

  rm_ecat_msgs::RmEcatStandardSlaveReadings getReadingsMsg();
  sensor_msgs::JointState getJointStateMsg();
  std::vector<sensor_msgs::Imu> getImuMsgs();

  void updateProcessReadings() override;
  void sendRos() override;

 protected:
  void startupRosInterface() override;
  void shutdownRosInterface() override;

  bool imuTriggerCallback(rm_msgs::EnableImuTrigger::Request& req, rm_msgs::EnableImuTrigger::Response& res);

  ros::Subscriber commandsSubscriber_;
  any_node::ThreadedPublisherPtr<rm_ecat_msgs::RmEcatStandardSlaveReadings> readingsPublisher_;
  any_node::ThreadedPublisherPtr<sensor_msgs::JointState> jointStatesPublisher_;
  std::vector<any_node::ThreadedPublisherPtr<sensor_msgs::Imu>> imuPublishers_;
  std::vector<any_node::ThreadedPublisherPtr<rm_msgs::DbusData>> dbusDatasPublishers_;
  std::vector<any_node::ThreadedPublisherPtr<rm_msgs::GpioData>> gpioDatasPublishers_;
  std::vector<ros::Publisher> imuTriggerTimePublishers_;
  ros::ServiceServer imusTriggerServer_;

  std::recursive_mutex readingsMsgMutex_;
  std::atomic<bool> readingsMsgUpdated_;
  rm_ecat_msgs::RmEcatStandardSlaveReadings readingsMsg_;

  std::recursive_mutex jointStatesMsgMutex_;
  std::atomic<bool> jointStatesMsgUpdated_;
  sensor_msgs::JointState jointStatesMsg_;

  std::recursive_mutex imuMsgsMutex_;
  std::atomic<bool> imuMsgsUpdated_;
  std::vector<sensor_msgs::Imu> imuMsgs_;

  std::recursive_mutex dbusDatasMutex_;
  std::atomic<bool> dbusDatasUpdated_;
  std::vector<rm_msgs::DbusData> dbusDatas;

 std::recursive_mutex gpioDatasMutex_;
 std::atomic<bool> gpioDatasUpdated_;
 std::vector<rm_msgs::GpioData> gpioDatas;
};
}  // namespace standard
}  // namespace rm_ecat
