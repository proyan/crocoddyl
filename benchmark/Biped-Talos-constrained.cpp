///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "all-robots-constrained.hpp"
#include "factory/robot-ee-names.hpp"

int main() {
  std::vector<std::string> contact_names;
  std::vector<crocoddyl::ContactType> contact_types;

  // Biped icub Benchmarks
  std::cout << "********************Biped iCub ***********************" << std::endl;
  contact_names.clear();
  contact_types.clear();
  contact_names.push_back("r_ankle_roll");
  contact_names.push_back("l_ankle_roll");
  contact_types.push_back(crocoddyl::Contact6D);
  contact_types.push_back(crocoddyl::Contact6D);

  RobotEENames bipedIcub(
      "iCub", contact_names, contact_types, EXAMPLE_ROBOT_DATA_MODEL_DIR "/icub_description/robots/icub_reduced.urdf",
      EXAMPLE_ROBOT_DATA_MODEL_DIR "/icub_description/srdf/icub.srdf", "r_wrist_yaw", "half_sitting");
  print_constrained_benchmark(bipedIcub);

  return 0;
}
