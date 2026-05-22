//
// Created by qiayuan on 23-4-13.
//

#include "rm_ecat_ros/RosMsgConversions.h"

namespace rm_ecat {

ros::Time createRosTime(const std::chrono::time_point<std::chrono::high_resolution_clock>& timePoint) {
  return {static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(timePoint.time_since_epoch()).count()),
          static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(timePoint.time_since_epoch()).count() % 1000000000)};
}

rm_ecat_msgs::RmEcatStandardSlaveReading createRmSlaveReadingMsg(const rm_ecat::standard::Reading& reading) {
  rm_ecat_msgs::RmEcatStandardSlaveReading readingMsg;
  readingMsg.stamp = createRosTime(reading.getStamp());
  readingMsg.statusword = reading.getStatusword().getRaw();

  auto ids = reading.getEnabledMotorIds(standard::CanBus::CAN0);
  for (const auto& id : ids) {
    readingMsg.names.push_back(reading.getMotorName(standard::CanBus::CAN0, id));
    readingMsg.isOnline.push_back(reading.getStatusword().isOnline(standard::CanBus::CAN0, id));
    //    readingMsg.isOverTemperature.push_back(reading.getStatusword().isOverTemperature(CanBus::CAN0, id));
    readingMsg.position.push_back(reading.getPosition(standard::CanBus::CAN0, id));
    readingMsg.velocity.push_back(reading.getVelocity(standard::CanBus::CAN0, id));
    readingMsg.torque.push_back(reading.getTorque(standard::CanBus::CAN0, id));
    readingMsg.temperature.push_back(reading.getTemperature(standard::CanBus::CAN0, id));
  }
  ids = reading.getEnabledMotorIds(standard::CanBus::CAN1);
  for (const auto& id : ids) {
    readingMsg.names.push_back(reading.getMotorName(standard::CanBus::CAN1, id));
    readingMsg.isOnline.push_back(reading.getStatusword().isOnline(standard::CanBus::CAN1, id));
    //    readingMsg.isOverTemperature.push_back(reading.getStatusword().isOverTemperature(CanBus::CAN1, id));
    readingMsg.position.push_back(reading.getPosition(standard::CanBus::CAN1, id));
    readingMsg.velocity.push_back(reading.getVelocity(standard::CanBus::CAN1, id));
    readingMsg.torque.push_back(reading.getTorque(standard::CanBus::CAN1, id));
    readingMsg.temperature.push_back(reading.getTemperature(standard::CanBus::CAN1, id));
  }

  return readingMsg;
}

rm_ecat_msgs::RmEcatStandardSlaveReadings createRmSlaveReadingsMsg(const std::vector<std::string>& names,
                                                                   const std::vector<rm_ecat::standard::Reading>& readings) {
  rm_ecat_msgs::RmEcatStandardSlaveReadings readingsMsg;
  for (size_t i = 0; i < names.size(); i++) {
    readingsMsg.names.push_back(names[i]);
    readingsMsg.readings.push_back(createRmSlaveReadingMsg(readings[i]));
  }

  return readingsMsg;
}

sensor_msgs::JointState createRmJointStateMsg(const std::vector<rm_ecat::standard::Reading>& readings) {
  sensor_msgs::JointState jointState;
  for (const auto& reading : readings) {
    auto ids = reading.getEnabledMotorIds(standard::CanBus::CAN0);
    for (const auto& id : ids) {
      jointState.name.push_back(reading.getMotorName(standard::CanBus::CAN0, id));
      jointState.position.push_back(reading.getPosition(standard::CanBus::CAN0, id));
      jointState.velocity.push_back(reading.getVelocity(standard::CanBus::CAN0, id));
      jointState.effort.push_back(reading.getTorque(standard::CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(standard::CanBus::CAN1);
    for (const auto& id : ids) {
      jointState.name.push_back(reading.getMotorName(standard::CanBus::CAN1, id));
      jointState.position.push_back(reading.getPosition(standard::CanBus::CAN1, id));
      jointState.velocity.push_back(reading.getVelocity(standard::CanBus::CAN1, id));
      jointState.effort.push_back(reading.getTorque(standard::CanBus::CAN1, id));
    }
  }

  return jointState;
}

sensor_msgs::JointState createMitJointStateMsg(const std::vector<rm_ecat::mit::Reading>& readings) {
  sensor_msgs::JointState jointState;
  for (const auto& reading : readings) {
    auto ids = reading.getEnabledMotorIds(mit::CanBus::CAN0);
    for (const auto& id : ids) {
      jointState.name.push_back(reading.getMotorName(mit::CanBus::CAN0, id));
      jointState.position.push_back(reading.getPosition(mit::CanBus::CAN0, id));
      jointState.velocity.push_back(reading.getVelocity(mit::CanBus::CAN0, id));
      jointState.effort.push_back(reading.getTorque(mit::CanBus::CAN0, id));
    }
    ids = reading.getEnabledMotorIds(mit::CanBus::CAN1);
    for (const auto& id : ids) {
      jointState.name.push_back(reading.getMotorName(mit::CanBus::CAN1, id));
      jointState.position.push_back(reading.getPosition(mit::CanBus::CAN1, id));
      jointState.velocity.push_back(reading.getVelocity(mit::CanBus::CAN1, id));
      jointState.effort.push_back(reading.getTorque(mit::CanBus::CAN1, id));
    }
  }

  return jointState;
}

rm_ecat_msgs::RmEcatMitSlaveReading createMitSlaveReadingMsg(const rm_ecat::mit::Reading& reading) {
  rm_ecat_msgs::RmEcatMitSlaveReading readingMsg;
  readingMsg.stamp = createRosTime(reading.getStamp());
  readingMsg.statusword = reading.getStatusword().getRaw();

  auto ids = reading.getEnabledMotorIds(mit::CanBus::CAN0);
  for (const auto& id : ids) {
    readingMsg.names.push_back(reading.getMotorName(mit::CanBus::CAN0, id));
    readingMsg.isOnline.push_back(reading.getStatusword().isOnline(mit::CanBus::CAN0, id));
    readingMsg.position.push_back(reading.getPosition(mit::CanBus::CAN0, id));
    readingMsg.velocity.push_back(reading.getVelocity(mit::CanBus::CAN0, id));
    readingMsg.torque.push_back(reading.getTorque(mit::CanBus::CAN0, id));
  }
  ids = reading.getEnabledMotorIds(mit::CanBus::CAN1);
  for (const auto& id : ids) {
    readingMsg.names.push_back(reading.getMotorName(mit::CanBus::CAN1, id));
    readingMsg.isOnline.push_back(reading.getStatusword().isOnline(mit::CanBus::CAN1, id));
    readingMsg.position.push_back(reading.getPosition(mit::CanBus::CAN1, id));
    readingMsg.velocity.push_back(reading.getVelocity(mit::CanBus::CAN1, id));
    readingMsg.torque.push_back(reading.getTorque(mit::CanBus::CAN1, id));
  }

  return readingMsg;
}

rm_ecat_msgs::RmEcatMitSlaveReadings createMitSlaveReadingsMsg(const std::vector<std::string>& names,
                                                               const std::vector<rm_ecat::mit::Reading>& readings) {
  rm_ecat_msgs::RmEcatMitSlaveReadings readingsMsg;
  for (size_t i = 0; i < names.size(); i++) {
    readingsMsg.names.push_back(names[i]);
    readingsMsg.readings.push_back(createMitSlaveReadingMsg(readings[i]));
  }

  return readingsMsg;
}

sensor_msgs::Imu createImuMsg(standard::CanBus bus, const rm_ecat::standard::Reading& reading) {
  sensor_msgs::Imu imuMsg;
  imuMsg.header.stamp = createRosTime(reading.getStamp());
  imuMsg.header.frame_id = reading.getImuName(bus);
  reading.getOrientation(bus, imuMsg.orientation.w, imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z);
  reading.getLinearAcceleration(bus, imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z);
  reading.getAngularVelocity(bus, imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z);
  return imuMsg;
}

std::vector<sensor_msgs::Imu> createImuMsgs(const std::vector<rm_ecat::standard::Reading>& readings) {
  std::vector<sensor_msgs::Imu> imuMsgs;
  for (const auto& reading : readings) {
    const auto imuBuss = reading.getEnabledImuBuss();
    for (const auto& bus : imuBuss) {
      imuMsgs.push_back(createImuMsg(bus, reading));
    }
  }
  return imuMsgs;
}

static ros::Time lastDbusStamp;
rm_msgs::DbusData createDbusData(const rm_ecat::standard::Reading& reading) {
  auto data = reading.getDbusData();
  if (reading.getDbusStatus()) {
    data.stamp = createRosTime(reading.getStamp());
    lastDbusStamp = data.stamp;
  }
  data.stamp = lastDbusStamp;
  return data;
}

std::vector<rm_msgs::DbusData> createDbusDatas(const std::vector<rm_ecat::standard::Reading>& readings)
{
  std::vector<rm_msgs::DbusData> dbusDatas;
  for (const auto& reading : readings) {
    if(reading.getEnabledDbus())
    {
      dbusDatas.push_back(createDbusData(reading));
    }
  }
  return dbusDatas;
}

rm_msgs::GpioData createRmGpioData(const rm_ecat::standard::Reading& reading) {
  rm_msgs::GpioData data;
  auto ids = reading.getEnabledDigitalInputIds();
  for (const auto& id : ids) {
    data.gpio_name.emplace_back(reading.getGpioName(id));
    data.gpio_state.push_back(reading.getDigitalInput(id));
    data.gpio_type.emplace_back("in");
  }
  data.header.stamp = createRosTime(reading.getStamp());
  return data;
}

rm_msgs::GpioData createMitGpioData(const rm_ecat::mit::Reading& reading) {
  rm_msgs::GpioData data;
  auto ids = reading.getEnabledDigitalInputIds();
  for (const auto& id : ids) {
    data.gpio_name.emplace_back(reading.getGpioName(id));
    data.gpio_state.push_back(reading.getDigitalInput(id));
    data.gpio_type.emplace_back("in");
  }
  data.header.stamp = createRosTime(reading.getStamp());
  return data;
}

std::vector<rm_msgs::GpioData> createRmGpioDatas(const std::vector<rm_ecat::standard::Reading>& readings) {
  std::vector<rm_msgs::GpioData> gpioDatas;
  gpioDatas.reserve(readings.size());
  for (const auto& reading : readings) {
      gpioDatas.push_back(createRmGpioData(reading));
  }
  return gpioDatas;
}

std::vector<rm_msgs::GpioData> createMitGpioDatas(const std::vector<rm_ecat::mit::Reading>& readings) {
  std::vector<rm_msgs::GpioData> gpioDatas;
  gpioDatas.reserve(readings.size());
  for (const auto& reading : readings) {
      gpioDatas.push_back(createMitGpioData(reading));
  }
  return gpioDatas;
}

sensor_msgs::TimeReference createTimeReferenceMsg(const std::chrono::time_point<std::chrono::high_resolution_clock>& timePoint) {
  sensor_msgs::TimeReference timeReferenceMsg;
  timeReferenceMsg.header.stamp = createRosTime(timePoint);
  timeReferenceMsg.time_ref = timeReferenceMsg.header.stamp;
  return timeReferenceMsg;
}

}  // namespace rm_ecat
