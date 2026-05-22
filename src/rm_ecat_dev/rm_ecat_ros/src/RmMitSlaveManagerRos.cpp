//
// Created by kook on 12/25/23.
//

#include "rm_ecat_ros/RmMitSlaveManagerRos.h"
#include "rm_ecat_ros/RosMsgConversions.h"

namespace rm_ecat {
namespace mit {
rm_ecat_msgs::RmEcatMitSlaveReadings RmMitSlaveManagerRos::getReadingsMsg() {
  std::lock_guard<std::recursive_mutex> readingsMsgLock(readingsMsgMutex_);
  return readingsMsg_;
}

sensor_msgs::JointState RmMitSlaveManagerRos::getJointStateMsg() {
  std::lock_guard<std::recursive_mutex> jointStatesMsgLock(jointStatesMsgMutex_);
  return jointStatesMsg_;
}

void RmMitSlaveManagerRos::updateProcessReadings() {
  ClearSlaveManagerRosBase<RmEcatMitManager>::updateProcessReadings();
  const auto& readings = getReadings<rm_ecat::mit::Reading>();
  {
    std::lock_guard<std::recursive_mutex> readingsMsgLock(readingsMsgMutex_);
    readingsMsg_ = createMitSlaveReadingsMsg(getNamesOfSlaves(), readings);
    if (readingsPublisher_) {
      readingsPublisher_->publish(readingsMsg_);
    }
    readingsMsgUpdated_ = true;
  }
  {
    std::lock_guard<std::recursive_mutex> readingsMsgLock(jointStatesMsgMutex_);
    jointStatesMsg_ = createMitJointStateMsg(readings);
    if (jointStatesPublisher_) {
      jointStatesPublisher_->publish(jointStatesMsg_);
    }
    jointStatesMsgUpdated_ = true;
  }
  {
    std::lock_guard<std::recursive_mutex> readingsMsgLock(gpioDatasMutex_);
    gpioDatas = createMitGpioDatas(readings);
    for (size_t i = 0; i < gpioDatasPublishers_.size(); ++i) {
      gpioDatasPublishers_[i]->publish(gpioDatas[i]);
    }
    gpioDatasUpdated_ = true;
  }
}

void RmMitSlaveManagerRos::sendRos() {
  if (readingsMsgUpdated_) {
    readingsMsgUpdated_ = false;
    readingsPublisher_->sendRos();
  }
  if (jointStatesMsgUpdated_) {
    jointStatesMsgUpdated_ = false;
    jointStatesPublisher_->sendRos();
  }
  if (gpioDatasUpdated_) {
    gpioDatasUpdated_ = false;
    for (auto& gpioPublisher : gpioDatasPublishers_) {
      gpioPublisher->sendRos();
    }
  }
}

void RmMitSlaveManagerRos::startupRosInterface() {
  shutdownPublishWorkerRequested_ = false;
  readingsPublisher_ = std::make_shared<any_node::ThreadedPublisher<rm_ecat_msgs::RmEcatMitSlaveReadings>>(
      nh_.advertise<rm_ecat_msgs::RmEcatMitSlaveReadings>("mit_readings", 10), 50, false);

  readingsMsgUpdated_ = false;

  jointStatesPublisher_ = std::make_shared<any_node::ThreadedPublisher<sensor_msgs::JointState>>(
      nh_.advertise<sensor_msgs::JointState>("joint_state", 10), 50, false);
  jointStatesMsgUpdated_ = false;

  const auto slaves = getSlaves();
  for (const auto& slave : slaves) {
    if (!slave->getReading().getEnabledDigitalInputIds().empty()) {
      gpioDatasPublishers_.push_back(std::make_shared<any_node::ThreadedPublisher<rm_msgs::GpioData>>(
          nh_.advertise<rm_msgs::GpioData>(slave->getName() + "_gpios", 10), 50, false));
    }
  }

  if (standalone_) {
    publishWorker_->start(20);
  }
}

void RmMitSlaveManagerRos::shutdownRosInterface() {
  if (!shutdownPublishWorkerRequested_) {
    jointStatesPublisher_->shutdown();
    readingsPublisher_->shutdown();
    for (auto& gpioPublisher : gpioDatasPublishers_) {
      gpioPublisher->shutdown();
    }
  }
}
}  // namespace mit
}  // namespace rm_ecat