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
  // Quadruped Solo Benchmarks
  std::cout << "********************Quadruped Solo******************" << std::endl;
  std::vector<std::string> contact_names;
  std::vector<crocoddyl::ContactType> contact_types;
  contact_names.clear();
  contact_types.clear();
  contact_names.push_back("FR_KFE");
  contact_names.push_back("HL_KFE");
  contact_types.push_back(crocoddyl::Contact3D);
  contact_types.push_back(crocoddyl::Contact3D);
  RobotEENames quadrupedSolo("Solo", contact_names, contact_types,
                             EXAMPLE_ROBOT_DATA_MODEL_DIR "/solo_description/robots/solo.urdf",
                             EXAMPLE_ROBOT_DATA_MODEL_DIR "/solo_description/srdf/solo.srdf", "HL_KFE", "standing");

  print_constrained_benchmark(quadrupedSolo);
  return 0;
}
