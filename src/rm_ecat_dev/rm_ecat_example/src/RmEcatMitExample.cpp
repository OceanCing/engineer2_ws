//
// Created by kook on 6/9/24.
//

#include <rm_ecat_manager/RmEcatMitManager.h>

rm_ecat::mit::RmEcatMitManager mitManager(true, true, 0.001);
auto busManager = std::make_shared<ecat_manager::EcatBusManager>();

bool controlUpdate(const any_worker::WorkerEvent& /*event*/) {
  auto slaves = mitManager.getSlaves();
  rm_ecat::mit::Command command;
  rm_ecat::mit::target target{3.14, 0., 0.2, 1.5, 0.};
  command.setTargetCommand(rm_ecat::mit::CanBus::CAN0, 1, target);

  slaves[0]->stageCommand(command);
  return true;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "pass path to 'setup.yaml' as command line argument" << std::endl;
    return EXIT_FAILURE;
  }

  busManager->fromFile(argv[1], false);
  mitManager.setBusManager(busManager);
  mitManager.startup();
  std::cout << "Startup finished" << std::endl;

  any_worker::Worker controlWorker("ControlWorker", mitManager.getTimeStep(), controlUpdate);
  controlWorker.start(47);

  while (mitManager.isRunning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  controlWorker.stop();
}
