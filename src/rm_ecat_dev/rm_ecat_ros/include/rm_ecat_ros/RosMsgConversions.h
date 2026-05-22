//
// Created by qiayuan on 23-4-13.
//

#pragma once

#include <rm_ecat_mit/Reading.h>
#include <rm_ecat_msgs/RmEcatMitSlaveReadings.h>
#include <rm_ecat_msgs/RmEcatStandardSlaveReadings.h>
#include <rm_ecat_standard_slave/Reading.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/TimeReference.h>

namespace rm_ecat {

ros::Time createRosTime(const std::chrono::time_point<std::chrono::high_resolution_clock>& timePoint);

rm_ecat_msgs::RmEcatStandardSlaveReading createRmSlaveReadingMsg(const rm_ecat::standard::Reading& reading);

rm_ecat_msgs::RmEcatStandardSlaveReadings createRmSlaveReadingsMsg(const std::vector<std::string>& names,
                                                                   const std::vector<rm_ecat::standard::Reading>& readings);

rm_ecat_msgs::RmEcatMitSlaveReading createMitSlaveReadingMsg(const rm_ecat::mit::Reading& reading);

rm_ecat_msgs::RmEcatMitSlaveReadings createMitSlaveReadingsMsg(const std::vector<std::string>& names,
                                                               const std::vector<rm_ecat::mit::Reading>& readings);

sensor_msgs::JointState createRmJointStateMsg(const std::vector<rm_ecat::standard::Reading>& readings);

sensor_msgs::JointState createMitJointStateMsg(const std::vector<rm_ecat::mit::Reading>& readings);

sensor_msgs::Imu createImuMsg(standard::CanBus bus, const rm_ecat::standard::Reading& reading);

std::vector<sensor_msgs::Imu> createImuMsgs(const std::vector<rm_ecat::standard::Reading>& readings);

rm_msgs::DbusData createDbusData(const rm_ecat::standard::Reading& reading);

std::vector<rm_msgs::DbusData> createDbusDatas(const std::vector<rm_ecat::standard::Reading>& readings);

rm_msgs::GpioData createRmGpioData(const rm_ecat::standard::Reading& reading);

std::vector<rm_msgs::GpioData> createRmGpioDatas(const std::vector<rm_ecat::standard::Reading>& readings);

rm_msgs::GpioData createMitGpioData(const rm_ecat::mit::Reading& reading);

std::vector<rm_msgs::GpioData> createMitGpioDatas(const std::vector<rm_ecat::mit::Reading>& readings);

sensor_msgs::TimeReference createTimeReferenceMsg(const std::chrono::time_point<std::chrono::high_resolution_clock>& timePoint);

}  // namespace rm_ecat
