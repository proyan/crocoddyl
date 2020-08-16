///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "all-robots.hpp"
#include "factory/robot-ee-names.hpp"

int main() {
  // Arm Manipulation Benchmarks
  std::cout << "********************  Kinova Arm  ******************" << std::endl;
  std::vector<std::string> contact_names;
  std::vector<crocoddyl::ContactType> contact_types;
  RobotEENames kinovaArm("Kinova_arm", contact_names, contact_types,
                         EXAMPLE_ROBOT_DATA_MODEL_DIR "/kinova_description/robots/kinova.urdf",
                         EXAMPLE_ROBOT_DATA_MODEL_DIR "/kinova_description/srdf/kinova.srdf", "gripper_left_joint",
                         "arm_up");

  print_benchmark(kinovaArm);
  
  return 0;
}
