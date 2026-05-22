//
// Created by kook on 12/25/23.
//

#include <ros/ros.h>
#include <mutex>

#include <rm_ecat_manager/RmEcatMitManager.h>
#include <rm_ecat_msgs/RmEcatMitSlaveReadings.h>
#include <sensor_msgs/JointState.h>
#include <any_node/ThreadedPublisher.hpp>

#include "rm_ecat_ros/ClearSlaveManagerRosBase.h"

namespace rm_ecat {
namespace mit {
class RmMitSlaveManagerRos : public cleardrive_ros::ClearSlaveManagerRosBase<rm_ecat::mit::RmEcatMitManager> {
 public:
  using ClearSlaveManagerRosBase<rm_ecat::mit::RmEcatMitManager>::ClearSlaveManagerRosBase;

  rm_ecat_msgs::RmEcatMitSlaveReadings getReadingsMsg();
  sensor_msgs::JointState getJointStateMsg();

  void updateProcessReadings() override;
  void sendRos() override;

 protected:
  void startupRosInterface() override;
  void shutdownRosInterface() override;

  ros::Subscriber commandsSubscriber_;
  any_node::ThreadedPublisherPtr<rm_ecat_msgs::RmEcatMitSlaveReadings> readingsPublisher_;
  any_node::ThreadedPublisherPtr<sensor_msgs::JointState> jointStatesPublisher_;
  std::vector<any_node::ThreadedPublisherPtr<rm_msgs::GpioData>> gpioDatasPublishers_;

  std::recursive_mutex readingsMsgMutex_;
  std::atomic<bool> readingsMsgUpdated_;
  rm_ecat_msgs::RmEcatMitSlaveReadings readingsMsg_;

  std::recursive_mutex jointStatesMsgMutex_;
  std::atomic<bool> jointStatesMsgUpdated_;
  sensor_msgs::JointState jointStatesMsg_;


  std::recursive_mutex gpioDatasMutex_;
  std::atomic<bool> gpioDatasUpdated_;
  std::vector<rm_msgs::GpioData> gpioDatas;
};
}  // namespace mit
}  // namespace rm_ecat
