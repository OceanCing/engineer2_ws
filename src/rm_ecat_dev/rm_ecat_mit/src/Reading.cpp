//
// Created by kook on 12/29/23.
//

#include "rm_ecat_mit/Reading.h"

#include <cmath>

namespace rm_ecat {
namespace mit {
Reading::Reading() {
  for (auto& velocityFilter : velocityFilters_) {
    velocityFilter = std::make_shared<LowPassFilter>(100);
  }
}

std::string Reading::getMotorName(CanBus bus, size_t id) const {
  return names_[getIndex(bus, id)];
}

std::vector<size_t> Reading::getEnabledMotorIds(CanBus bus) const {
  std::vector<size_t> enabledMotorIds;
  for (size_t id = 1; id <= motorNumEachBus; ++id) {
    if (isMotorEnabled_[getIndex(bus, id)]) {
      enabledMotorIds.push_back(id);
    }
  }
  return enabledMotorIds;
}

std::vector<uint8_t> Reading::getEnabledDigitalInputIds() const {
  std::vector<uint8_t> ids;
  for (uint8_t id = 0; id < 8; ++id) {
    if (isDigitalInputEnabled_[id]) {
      ids.push_back(id);
    }
  }
  return ids;
}

void Reading::configureReading(const Configuration& configuration) {
  for (const auto& [id, motorConfiguration] : configuration.can0MotorConfigurations_) {
    size_t index = getIndex(CanBus::CAN0, id);
    names_[index] = motorConfiguration.name_;
    isMotorEnabled_[index] = true;
    positionOffset[index] = motorConfiguration.positionOffset;
    velocityOffset[index] = motorConfiguration.velocityOffset;
    torqueOffset[index] = motorConfiguration.torqueOffset;
    positionFactorIntegerToRad_[index] = motorConfiguration.positionFactorIntegerToRad_;
    velocityFactorIntegerPerMinusToRadPerSec_[index] = motorConfiguration.velocityFactorIntegerPerMinusToRadPerSec_;
    torqueFactorIntegerToNm_[index] = motorConfiguration.torqueFactorIntegerToNm_;
  }
  for (const auto& [id, motorConfiguration] : configuration.can1MotorConfigurations_) {
    size_t index = getIndex(CanBus::CAN1, id);
    names_[index] = motorConfiguration.name_;
    isMotorEnabled_[index] = true;
    positionOffset[index] = motorConfiguration.positionOffset;
    velocityOffset[index] = motorConfiguration.velocityOffset;
    torqueOffset[index] = motorConfiguration.torqueOffset;
    positionFactorIntegerToRad_[index] = motorConfiguration.positionFactorIntegerToRad_;
    velocityFactorIntegerPerMinusToRadPerSec_[index] = motorConfiguration.velocityFactorIntegerPerMinusToRadPerSec_;
    torqueFactorIntegerToNm_[index] = motorConfiguration.torqueFactorIntegerToNm_;
  }
  for (const auto& [id, gpioConfiguration] : configuration.gpioConfigurations_) {
    if (gpioConfiguration.mode_ == 0) {
      isDigitalInputEnabled_[id] = true;
      gpioNames_[id] = gpioConfiguration.name_;
    }
  }
}

double Reading::getPosition(CanBus bus, size_t id) const {
  //  static double actualPosition_[motorNumEachBus * 2]{0};
  //  static int64_t actualCircle_[motorNumEachBus * 2]{0};
  //  static bool firstReceived_[motorNumEachBus * 2]{true};
  //  size_t index = getIndex(bus, id);
  //  uint16_t position = ((rawReadings_[index] >> 8 & 0xFF) << 8) | (rawReadings_[index] >> 16 & 0xFF);
  //  if (!firstReceived_[index])  // not the first receive
  //  {
  //    double pos_new = static_cast<double>(position * positionFactorIntegerToRad_[index] + positionOffset[index]) +
  //                     static_cast<double>(actualCircle_[index]) * 32 * M_PI;
  //    if (pos_new - actualPosition_[index] > 16 * M_PI) {
  //      actualCircle_[index]--;
  //    } else if (pos_new - actualPosition_[index] < -16 * M_PI) {
  //      actualCircle_[index]++;
  //    }
  //  } else {
  //    actualCircle_[index] = 0;
  //    firstReceived_[index] = false;
  //  }
  //  actualPosition_[index] = static_cast<double>(position * positionFactorIntegerToRad_[index] + positionOffset[index] +
  //                                               static_cast<double>(actualCircle_[index]) * 32 * M_PI);
  //  return actualPosition_[index];
  size_t index = getIndex(bus, id);
  uint16_t position = ((rawReadings_[index] >> 8 & 0xFF) << 8) | (rawReadings_[index] >> 16 & 0xFF);
  return static_cast<double>(position * positionFactorIntegerToRad_[index] + positionOffset[index]);
}

double Reading::getVelocity(CanBus bus, size_t id) const {
  size_t index = getIndex(bus, id);
  uint16_t velocity = ((rawReadings_[index] >> 24 & 0xFF) << 4) | (rawReadings_[index] >> 36 & 0xF);

  //  double time = getTime(stamp_);

  //  velocityFilters_[i]->input(velocity, time);
  //  velocity = velocityFilters_[i]->output();
  return static_cast<double>(velocity * velocityFactorIntegerPerMinusToRadPerSec_[index] + velocityOffset[index]);
}

double Reading::getTorque(CanBus bus, size_t id) const {
  size_t index = getIndex(bus, id);
  uint16_t current = ((rawReadings_[index] >> 32 & 0xF) << 8) | (rawReadings_[index] >> 40 & 0xFF);
  return static_cast<double>(current * torqueFactorIntegerToNm_[index] + torqueOffset[index]);
}

uint16_t Reading::getRawReading(CanBus bus, size_t id) const {
  return rawReadings_[getIndex(bus, id)];
}

bool Reading::getDigitalInput(uint8_t id) const {
  return digitalInputs_[id];
}

void Reading::setRawReading(CanBus bus, size_t id, uint64_t data) {
  rawReadings_[getIndex(bus, id)] = data;
}

void Reading::setDigitalInputs(uint8_t value) {
  for (size_t i = 0; i < 8; ++i) {
    digitalInputs_[i] = ((value & (1 << i)) != 0);
  }
}

std::string Reading::getGpioName(uint8_t id) const{
  return gpioNames_[id];
}

}  // namespace mit
}  // namespace rm_ecat
