//
// Created by qiayuan on 23-4-16.
//

#include "rm_ecat_ros/RmEcatHardwareInterface.h"
#include "rm_ecat_ros/RosMsgConversions.h"

#include <joint_limits_interface/joint_limits_urdf.h>

namespace rm_ecat {
bool RmEcatHardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  std::string name = "rm_ecat_hw";
  rmStandardSlaveManager_ = std::make_shared<rm_ecat::standard::RmStandardSlaveManagerRos>(false, false, 0.001, robot_hw_nh, name);
  rmMitSlaveManager_ = std::make_shared<rm_ecat::mit::RmMitSlaveManagerRos>(false, false, 0.001, robot_hw_nh, name);
  std::string setupFile;
  robot_hw_nh.getParam("setupFile", setupFile);

  busManager_ = std::make_shared<ecat_manager::EcatBusManager>();
  busManager_->fromFile(setupFile, false);
  busManager_->startupCommunication();
  rmStandardSlaveManager_->setBusManager(busManager_);
  rmMitSlaveManager_->setBusManager(busManager_);

  busNames_ = busManager_->getAllBusNames();
  for (const auto& busName : busNames_) {
    busIsOk_.emplace(busName, true);
  }
  busStatesPublisher_ = std::make_shared<any_node::ThreadedPublisher<rm_msgs::BusState>>(
      robot_hw_nh.advertise<rm_msgs::BusState>("bus_state", 10), 10, false);
  if (!rmStandardSlaveManager_->getDigitalOutputNames().empty()) {
    rmGpioOutputsPublisher_ = std::make_shared<any_node::ThreadedPublisher<rm_msgs::GpioData>>(
        robot_hw_nh.advertise<rm_msgs::GpioData>("gpio_expect_outputs/rm", 10), 20, false);
  }
  if (!rmMitSlaveManager_->getDigitalOutputNames().empty()) {
    mitGpioOutputsPublisher_ = std::make_shared<any_node::ThreadedPublisher<rm_msgs::GpioData>>(
        robot_hw_nh.advertise<rm_msgs::GpioData>("gpio_expect_outputs/mit", 10), 20, false);
  }

  controllerManager_ = std::make_shared<controller_manager::ControllerManager>(this, root_nh);

  if (!rmStandardSlaveManager_->startup()) {
    return false;
  }
  if (!rmMitSlaveManager_->startup()) {
    return false;
  }

  updateWorker_ = std::make_shared<any_worker::Worker>("updateWorker", rmStandardSlaveManager_->getTimeStep(),
                                                       std::bind(&RmEcatHardwareInterface::updateWorkerCb, this, std::placeholders::_1));
  controlWorker_ = std::make_shared<any_worker::Worker>("controlWorker", rmStandardSlaveManager_->getTimeStep(),
                                                        std::bind(&RmEcatHardwareInterface::controlWorkerCb, this, std::placeholders::_1));
  publishWorker_ = std::make_shared<any_worker::Worker>("PublishWorker", 10 * rmStandardSlaveManager_->getTimeStep(),
                                                        std::bind(&RmEcatHardwareInterface::publishWorkerCb, this, std::placeholders::_1));
  signal_handler::SignalHandler::bindAll(&RmEcatHardwareInterface::handleSignal, this);

  ////////// ros-control //////////
  if (!loadUrdf(root_nh)) {
    MELO_ERROR("Error occurred while setting up urdf");
    return false;
  }
  setupActuators();
  if (!setupTransmission()) {
    MELO_ERROR("Error occurred while setting up transmission");
    return false;
  }
  if (!setupJointLimit()) {
    MELO_ERROR("Error occurred while setting up joint limit");
    return false;
  }
  setupImus();
  setupGpios();

  registerInterface(&robot_state_interface_);

  updateWorker_->start(48);
  controlWorker_->start(48);
  publishWorker_->start(20);

  return true;
}

bool RmEcatHardwareInterface::isRunning() {
  return rmStandardSlaveManager_->isRunning() && rmMitSlaveManager_->isRunning();
}

bool RmEcatHardwareInterface::controlWorkerCb(const any_worker::WorkerEvent& event) {
  ros::Time time = ros::Time::now();  // TODO: how can we use event.timeStamp?
  ros::Duration period(event.timeStep);

  read(time, period);
  controllerManager_->update(time, period);
  write(time, period);

  return true;
}

bool RmEcatHardwareInterface::updateWorkerCb(const any_worker::WorkerEvent& event) {
  busManager_->readAllBuses();
  rmStandardSlaveManager_->updateProcessReadings();
  rmMitSlaveManager_->updateProcessReadings();
  rmStandardSlaveManager_->updateSendStagedCommands();
  rmMitSlaveManager_->updateSendStagedCommands();
  busManager_->writeToAllBuses();
  return true;
}

bool RmEcatHardwareInterface::publishWorkerCb(const any_worker::WorkerEvent& /*unused*/) {
  rmStandardSlaveManager_->sendRos();
  rmMitSlaveManager_->sendRos();
  if (busStatesMsgUpdated_) {
    busStatesMsgUpdated_ = false;
    if (busStatesPublisher_) {
      busStatesPublisher_->sendRos();
    }
  }
  if (rmGpioOutputsMsgUpdated_) {
    rmGpioOutputsMsgUpdated_ = false;
    if (rmGpioOutputsPublisher_) {
      rmGpioOutputsPublisher_->sendRos();
    }
  }
  if (mitGpioOutputsMsgUpdated_) {
    mitGpioOutputsMsgUpdated_ = false;
    if (mitGpioOutputsPublisher_) {
      mitGpioOutputsPublisher_->sendRos();
    }
  }
  return true;
}

void RmEcatHardwareInterface::handleSignal(int /*signum*/) {
  updateWorker_->stop();
  publishWorker_->stop();
  controlWorker_->stop();
  rmStandardSlaveManager_->shutdown();
  rmMitSlaveManager_->shutdown();
  busStatesPublisher_->shutdown();
  rmGpioOutputsPublisher_->shutdown();
  mitGpioOutputsPublisher_->shutdown();
}

bool RmEcatHardwareInterface::loadUrdf(ros::NodeHandle& root_nh) {
  if (urdf_model_ == nullptr) {
    urdf_model_ = std::make_shared<urdf::Model>();
  }
  // get the urdf param on param server
  root_nh.getParam("/robot_description", urdf_string_);
  return !urdf_string_.empty() && urdf_model_->initString(urdf_string_);
}

void RmEcatHardwareInterface::setupActuators() {
  // rm standard slave
  size_t num_motors = rmStandardSlaveManager_->getMotorNames().size();
  auto motorNames = rmStandardSlaveManager_->getMotorNames();
  auto motorNeedCalibrations = rmStandardSlaveManager_->getMotorNeedCalibrations();

  for (size_t i = 0; i < num_motors; ++i) {
    act_data_list_.push_back({0., 0., 0., 0., 0., 0., false, motorNeedCalibrations[i], false});
    hardware_interface::ActuatorStateHandle act_state(motorNames[i], &act_data_list_.back().pos, &act_data_list_.back().vel,
                                                      &act_data_list_.back().effort);
    rm_control::ActuatorExtraHandle act_ext(motorNames[i], &act_data_list_.back().halted, &act_data_list_.back().needCalibration,
                                            &act_data_list_.back().calibrated, &act_data_list_.back().calibrationReading,
                                            &act_data_list_.back().pos, &act_data_list_.back().offset);
    act_state_interface_.registerHandle(act_state);
    act_extra_interface_.registerHandle(act_ext);
    effort_act_interface_.registerHandle(hardware_interface::ActuatorHandle(act_state, &act_data_list_.back().command));
  }

  // mit slave
  num_motors = rmMitSlaveManager_->getMotorNames().size();
  motorNames = rmMitSlaveManager_->getMotorNames();
  motorNeedCalibrations = rmMitSlaveManager_->getMotorNeedCalibrations();

  for (size_t i = 0; i < num_motors; ++i) {
    act_data_list_.push_back({0., 0., 0., 0., 0., 0., false, motorNeedCalibrations[i], false});
    hardware_interface::ActuatorStateHandle act_state(motorNames[i], &act_data_list_.back().pos, &act_data_list_.back().vel,
                                                      &act_data_list_.back().effort);
    rm_control::ActuatorExtraHandle act_ext(motorNames[i], &act_data_list_.back().halted, &act_data_list_.back().needCalibration,
                                            &act_data_list_.back().calibrated, &act_data_list_.back().calibrationReading,
                                            &act_data_list_.back().pos, &act_data_list_.back().offset);
    act_state_interface_.registerHandle(act_state);
    act_extra_interface_.registerHandle(act_ext);
    effort_act_interface_.registerHandle(hardware_interface::ActuatorHandle(act_state, &act_data_list_.back().command));
  }

  // register interface
  registerInterface(&act_state_interface_);
  registerInterface(&act_extra_interface_);
  registerInterface(&effort_act_interface_);
}

bool RmEcatHardwareInterface::setupTransmission() {
  try {
    transmission_loader_ = std::make_unique<transmission_interface::TransmissionInterfaceLoader>(this, &robot_transmissions_);
  } catch (const std::invalid_argument& ex) {
    MELO_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return false;
  } catch (const pluginlib::LibraryLoadException& ex) {
    MELO_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return false;
  } catch (...) {
    MELO_ERROR_STREAM("Failed to create transmission interface loader. ");
    return false;
  }

  // Perform transmission loading
  if (!transmission_loader_->load(urdf_string_)) {
    return false;
  }
  act_to_jnt_state_ = robot_transmissions_.get<transmission_interface::ActuatorToJointStateInterface>();
  jnt_to_act_effort_ = robot_transmissions_.get<transmission_interface::JointToActuatorEffortInterface>();

  auto* effort_joint_interface = this->get<hardware_interface::EffortJointInterface>();
  std::vector<std::string> names = effort_joint_interface->getNames();
  for (const auto& name : names) {
    effort_joint_handles_.push_back(effort_joint_interface->getHandle(name));
  }
  return true;
}

bool RmEcatHardwareInterface::setupJointLimit() {
  joint_limits_interface::JointLimits joint_limits;     // Position
  joint_limits_interface::SoftJointLimits soft_limits;  // Soft Position

  for (const auto& joint_handle : effort_joint_handles_) {
    bool has_joint_limits{}, has_soft_limits{};
    std::string name = joint_handle.getName();
    // Get limits from URDF
    urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(name);
    if (urdf_joint == nullptr) {
      MELO_ERROR_STREAM("URDF joint not found " << name);
      return false;
    }
    // Get limits from URDF
    if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits)) {
      has_joint_limits = true;
      MELO_DEBUG_STREAM("Joint " << name << " has URDF position limits.");
    } else if (urdf_joint->type != urdf::Joint::CONTINUOUS) {
      MELO_DEBUG_STREAM("Joint " << name << " does not have a URDF limit.");
    }
    // Get soft limits from URDF
    if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) {
      has_soft_limits = true;
      MELO_DEBUG_STREAM("Joint " << name << " has soft joint limits from URDF.");
    } else {
      MELO_DEBUG_STREAM("Joint " << name << " does not have soft joint limits from URDF.");
    }

    // Slightly reduce the joint limits to prevent floating point errors
    if (joint_limits.has_position_limits) {
      joint_limits.min_position += std::numeric_limits<double>::epsilon();
      joint_limits.max_position -= std::numeric_limits<double>::epsilon();
    }
    if (has_soft_limits) {  // Use soft limits
      MELO_DEBUG_STREAM("Using soft saturation limits");
      effort_jnt_soft_limits_interface_.registerHandle(
          joint_limits_interface::EffortJointSoftLimitsHandle(joint_handle, joint_limits, soft_limits));
    } else if (has_joint_limits) {
      MELO_DEBUG_STREAM("Using saturation limits (not soft limits)");
      effort_jnt_saturation_interface_.registerHandle(joint_limits_interface::EffortJointSaturationHandle(joint_handle, joint_limits));
    }
  }
  return true;
}

void RmEcatHardwareInterface::setupImus() {
  size_t num_imus = rmStandardSlaveManager_->getImuNames().size();
  const auto imuNames = rmStandardSlaveManager_->getImuNames();
  for (size_t i = 0; i < num_imus; ++i) {
    imu_data_list_.push_back({});
    hardware_interface::ImuSensorHandle imu_handle(imuNames[i], imuNames[i], imu_data_list_.back().orientation,
                                                   imu_data_list_.back().orientationCovariance, imu_data_list_.back().angularVel,
                                                   imu_data_list_.back().angularVelocityCovariance, imu_data_list_.back().linearAccel,
                                                   imu_data_list_.back().linearAccelerationCovariance);
    rm_control::RmImuSensorHandle rm_imu_handle(imu_handle, &imu_data_list_.back().stamp);

    imu_sensor_interface_.registerHandle(imu_handle);
    rm_imu_sensor_interface_.registerHandle(rm_imu_handle);
  }
  registerInterface(&imu_sensor_interface_);
  registerInterface(&rm_imu_sensor_interface_);
}

void RmEcatHardwareInterface::setupGpios() {
  size_t num_inputs = rmStandardSlaveManager_->getDigitalInputNames().size();
  auto DigitalInputNames = rmStandardSlaveManager_->getDigitalInputNames();
  for (size_t i = 0; i < num_inputs; ++i) {
    digital_input_list_.push_back({});
    rm_control::GpioStateHandle gpio_state_handle(DigitalInputNames[i], rm_control::INPUT, &digital_input_list_.back());
    gpio_state_interface_.registerHandle(gpio_state_handle);
  }
  num_inputs = rmMitSlaveManager_->getDigitalInputNames().size();
  DigitalInputNames = rmMitSlaveManager_->getDigitalInputNames();
  for (size_t i = 0; i < num_inputs; ++i) {
    digital_input_list_.push_back({});
    rm_control::GpioStateHandle gpio_state_handle(DigitalInputNames[i], rm_control::INPUT, &digital_input_list_.back());
    gpio_state_interface_.registerHandle(gpio_state_handle);
  }

  size_t num_outputs = rmStandardSlaveManager_->getDigitalOutputNames().size();
  auto DigitalOutputNames = rmStandardSlaveManager_->getDigitalOutputNames();
  for (size_t i = 0; i < num_outputs; ++i) {
    digital_output_list_.push_back({});
    rm_control::GpioStateHandle gpio_state_handle(DigitalOutputNames[i], rm_control::OUTPUT, &digital_output_list_.back());
    rm_control::GpioCommandHandle gpio_command_handle(DigitalOutputNames[i], rm_control::OUTPUT, &digital_output_list_.back());
    gpio_state_interface_.registerHandle(gpio_state_handle);
    gpio_command_interface_.registerHandle(gpio_command_handle);
  }
  num_outputs = rmMitSlaveManager_->getDigitalOutputNames().size();
  DigitalOutputNames = rmMitSlaveManager_->getDigitalOutputNames();
  for (size_t i = 0; i < num_outputs; ++i) {
    digital_output_list_.push_back({});
    rm_control::GpioStateHandle gpio_state_handle(DigitalOutputNames[i], rm_control::OUTPUT, &digital_output_list_.back());
    rm_control::GpioCommandHandle gpio_command_handle(DigitalOutputNames[i], rm_control::OUTPUT, &digital_output_list_.back());
    gpio_state_interface_.registerHandle(gpio_state_handle);
    gpio_command_interface_.registerHandle(gpio_command_handle);
  }
  registerInterface(&gpio_state_interface_);
  registerInterface(&gpio_command_interface_);
}

void RmEcatHardwareInterface::read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // Motors
  const auto& rmMotorIsOnlines = rmStandardSlaveManager_->getMotorIsOnlines();
  const auto& rmMotorPositions = rmStandardSlaveManager_->getMotorPositions();
  const auto& rmMotorVelocities = rmStandardSlaveManager_->getMotorVelocities();
  const auto& rmMotorTorques = rmStandardSlaveManager_->getMotorTorque();

  size_t num_motors = rmStandardSlaveManager_->getMotorNames().size();
  auto slave_it = act_data_list_.begin();
  for (size_t i = 0; i < num_motors; ++i) {
    slave_it->halted = !rmMotorIsOnlines.at(i);  // TODO: add isOverTemperature
    slave_it->pos = rmMotorPositions.at(i) + slave_it->offset;
    slave_it->vel = rmMotorVelocities.at(i);
    slave_it->effort = rmMotorTorques.at(i);
    slave_it++;
  }

  rmMitSlaveManager_->checkMotorsIsonline();
  const auto& mitMotorIsOnlines = rmMitSlaveManager_->getMotorIsOnlines();
  const auto& mitMotorPositions = rmMitSlaveManager_->getMotorPositions();
  const auto& mitMotorVelocities = rmMitSlaveManager_->getMotorVelocities();
  const auto& mitMotorTorques = rmMitSlaveManager_->getMotorTorque();
  //
  num_motors = rmMitSlaveManager_->getMotorNames().size();
  for (size_t i = 0; i < num_motors; ++i) {
    slave_it->halted = !mitMotorIsOnlines.at(i);
    slave_it->pos = mitMotorPositions.at(i) + slave_it->offset;
    slave_it->vel = mitMotorVelocities.at(i);
    slave_it->effort = mitMotorTorques.at(i);
    slave_it++;
  }

  act_to_jnt_state_->propagate();
  for (auto& effort_joint_handle : effort_joint_handles_) {
    effort_joint_handle.setCommand(0.);  // Set all cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  }
  // Imus
  const auto imuOrientations = rmStandardSlaveManager_->getImuOrientations();
  const auto imuAngularVelocities = rmStandardSlaveManager_->getImuAngularVelocities();
  const auto imuLinearAccelerations = rmStandardSlaveManager_->getImuLinearAccelerations();
  const auto readings = rmStandardSlaveManager_->getReadings<rm_ecat::standard::Reading>();
  size_t i = 0;
  for (auto& imu : imu_data_list_) {
    imu.orientation[0] = imuOrientations.at(i * 4 + 0);
    imu.orientation[1] = imuOrientations.at(i * 4 + 1);
    imu.orientation[2] = imuOrientations.at(i * 4 + 2);
    imu.orientation[3] = imuOrientations.at(i * 4 + 3);
    imu.angularVel[0] = imuAngularVelocities.at(i * 3 + 0);
    imu.angularVel[1] = imuAngularVelocities.at(i * 3 + 1);
    imu.angularVel[2] = imuAngularVelocities.at(i * 3 + 2);
    imu.linearAccel[0] = imuLinearAccelerations.at(i * 3 + 0);
    imu.linearAccel[1] = imuLinearAccelerations.at(i * 3 + 1);
    imu.linearAccel[2] = imuLinearAccelerations.at(i * 3 + 2);
    imu.stamp = createRosTime(readings.at(0).getStamp());  // TODO: correct readings index
    ++i;
  }
  // Digital Inputs
  size_t num_inputs = rmStandardSlaveManager_->getDigitalInputs().size();
  auto digitalInputs = rmStandardSlaveManager_->getDigitalInputs();
  auto input_it = digital_input_list_.begin();
  for (i = 0; i < num_inputs; ++i) {
    *input_it = digitalInputs.at(i);
    input_it++;
  }
  num_inputs = rmMitSlaveManager_->getDigitalInputs().size();
  digitalInputs = rmMitSlaveManager_->getDigitalInputs();
  for (i = 0; i < num_inputs; ++i) {
    *input_it = digitalInputs.at(i);
    input_it++;
  }
}

void RmEcatHardwareInterface::write(const ros::Time& /*time*/, const ros::Duration& period) {
  // Propagate without joint limits
  jnt_to_act_effort_->propagate();
  // Save command before enforceLimits
  for (auto& act_data : act_data_list_) {
    act_data.commandUnlimited = act_data.command;
  }
  // enforceLimits will limit cmd_effort into suitable value https://github.com/ros-controls/ros_control/wiki/joint_limits_interface
  effort_jnt_saturation_interface_.enforceLimits(period);
  effort_jnt_soft_limits_interface_.enforceLimits(period);
  // Propagate with joint limits
  jnt_to_act_effort_->propagate();
  // Restore the command for the calibrating joint
  for (auto& act_data : act_data_list_) {
    if (act_data.needCalibration && !act_data.calibrated) {
      act_data.command = act_data.commandUnlimited;
    }
  }

  // Set command to motor
  std::vector<double> rm_slave_commands;
  size_t num_motors = rmStandardSlaveManager_->getMotorNames().size();
  auto slave_it = act_data_list_.begin();
  for (size_t i = 0; i < num_motors; ++i) {
    rm_slave_commands.push_back(slave_it->command);
    slave_it++;
  }
  rmStandardSlaveManager_->stageMotorCommands(rm_slave_commands);

  std::vector<rm_ecat::mit::target> mit_slave_commands;
  num_motors = rmMitSlaveManager_->getMotorNames().size();
  for (size_t i = 0; i < num_motors; ++i) {
    mit::target target{0., 0., slave_it->command, 0., 0.};
    mit_slave_commands.push_back(target);
    slave_it++;
  }
  rmMitSlaveManager_->stageMotorCommands(mit_slave_commands);

  // Digital outputs
  std::vector<bool> rm_digital_outputs;
  rm_msgs::GpioData rm_gpio_datas;
  auto rm_output_names = rmStandardSlaveManager_->getDigitalOutputNames();
  size_t rm_num_outputs = rmStandardSlaveManager_->getDigitalOutputNames().size();
  auto output_it = digital_output_list_.begin();
  for (size_t i = 0; i < rm_num_outputs; ++i) {
    rm_digital_outputs.push_back(*output_it);
    rm_gpio_datas.gpio_name.emplace_back(rm_output_names.at(i));
    rm_gpio_datas.gpio_state.push_back(*output_it);
    rm_gpio_datas.gpio_type.emplace_back("output");
    output_it++;
  }
  rm_gpio_datas.header.stamp = ros::Time::now();
  if (rmGpioOutputsPublisher_) {
    rmGpioOutputsPublisher_->publish(rm_gpio_datas);
  }
  rmGpioOutputsMsgUpdated_ = true;
  rmStandardSlaveManager_->stageDigitalOutputs(rm_digital_outputs);

  std::vector<bool> mit_digital_outputs;
  rm_msgs::GpioData mit_gpio_datas;
  auto mit_output_names = rmMitSlaveManager_->getDigitalOutputNames();
  size_t mit_num_outputs = rmMitSlaveManager_->getDigitalOutputNames().size();
  for (size_t i = 0; i < mit_num_outputs; ++i) {
    mit_digital_outputs.push_back(*output_it);
    mit_gpio_datas.gpio_name.emplace_back(mit_output_names.at(i));
    mit_gpio_datas.gpio_state.push_back(*output_it);
    mit_gpio_datas.gpio_type.emplace_back("output");
    output_it++;
  }
  mit_gpio_datas.header.stamp = ros::Time::now();
  if (mitGpioOutputsPublisher_) {
    mitGpioOutputsPublisher_->publish(mit_gpio_datas);
  }
  mitGpioOutputsMsgUpdated_ = true;
  rmMitSlaveManager_->stageDigitalOutputs(mit_digital_outputs);

  // bus monitoring
  if (busDiagDecimationCount_ > 100) {
    rm_msgs::BusState busStatesMsg_;
    for (const auto& bus : busNames_) {
      if (!busIsOk_.at(bus)) {
        if (busManager_->onActivate(bus)) {
          busIsOk_.at(bus) = true;
        }
      }
      if (!busManager_->busMonitoring(bus) && busIsOk_.at(bus)) {
        ROS_INFO("Bus is not ok");
        busManager_->onDeactivate(bus);
        busIsOk_.at(bus) = false;
      }
      busStatesMsg_.name.push_back(bus);
      busStatesMsg_.isOnline.push_back(busIsOk_.at(bus));
      busStatesMsg_.stamp = ros::Time::now();
    }
    if (busStatesPublisher_) {
      busStatesPublisher_->publish(busStatesMsg_);
    }
    busStatesMsgUpdated_ = true;
    busDiagDecimationCount_ = 0;
  }
  busDiagDecimationCount_++;
}

}  // namespace rm_ecat
