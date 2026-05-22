//
// Created by qiayuan on 23-4-12.
//

#include <rm_ecat_manager/RmEcatStandardSlaveManager.h>

rm_ecat::standard::RmEcatStandardSlaveManager rmManager(true, true, 0.001);
auto busManager = std::make_shared<ecat_manager::EcatBusManager>();

bool controlUpdate(const any_worker::WorkerEvent& /*event*/) {
  auto slaves = rmManager.getSlaves();
  rm_ecat::standard::Command command;

  rm_ecat::standard::Reading reading = slaves[0]->getReading();
  command.setTargetCommand(rm_ecat::standard::CanBus::CAN0, 2, 0.0006 * (0. - reading.getVelocity(rm_ecat::standard::CanBus::CAN0, 2)));

  slaves[0]->stageCommand(command);
  return true;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "pass path to 'setup.yaml' as command line argument" << std::endl;
    return EXIT_FAILURE;
  }

  busManager->fromFile(argv[1], false);
  rmManager.setBusManager(busManager);
  rmManager.startup();
  std::cout << "Startup finished" << std::endl;

  any_worker::Worker controlWorker("ControlWorker", rmManager.getTimeStep(), controlUpdate);
  controlWorker.start(47);

  while (rmManager.isRunning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  controlWorker.stop();
}
