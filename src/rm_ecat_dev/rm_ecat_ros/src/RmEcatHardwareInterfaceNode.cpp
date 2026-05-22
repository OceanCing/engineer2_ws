//
// Created by qiayuan on 23-4-16.
//

#include "rm_ecat_ros/RmEcatHardwareInterface.h"

rm_ecat::RmEcatHardwareInterface rmEcatHardwareInterface;

int main(int argc, char** argv) {
  std::string name = "rm_ecat_hw";
  ros::init(argc, argv, name);
  ros::NodeHandle nh("");
  ros::NodeHandle nhP("~");

  rmEcatHardwareInterface.init(nh, nhP);
  ros::spin();
}
