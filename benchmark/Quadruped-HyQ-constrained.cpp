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

  // Quadruped HyQ Benchmarks
  std::cout << "******************** Quadruped HyQ ******************" << std::endl;
  contact_names.clear();
  contact_types.clear();
  contact_names.push_back("rf_kfe_joint");
  contact_names.push_back("lf_kfe_joint");
  contact_names.push_back("lh_kfe_joint");
  contact_types.push_back(crocoddyl::Contact3D);
  contact_types.push_back(crocoddyl::Contact3D);
  contact_types.push_back(crocoddyl::Contact3D);
  RobotEENames quadrupedHyQ("HyQ", contact_names, contact_types,
                            EXAMPLE_ROBOT_DATA_MODEL_DIR "/hyq_description/robots/hyq_no_sensors.urdf",
                            EXAMPLE_ROBOT_DATA_MODEL_DIR "/hyq_description/srdf/hyq.srdf", "rh_kfe_joint", "standing");

  print_constrained_benchmark(quadrupedHyQ);

  return 0;
}
