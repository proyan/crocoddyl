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
  // Quadruped Anymal Benchmarks
  std::cout << "********************Quadruped Anymal******************" << std::endl;
  std::vector<std::string> contact_names;
  std::vector<crocoddyl::ContactType> contact_types;
  contact_names.clear();
  contact_types.clear();
  contact_names.push_back("RF_KFE");
  contact_names.push_back("LF_KFE");
  contact_names.push_back("LH_KFE");
  contact_types.push_back(crocoddyl::Contact3D);
  contact_types.push_back(crocoddyl::Contact3D);
  contact_types.push_back(crocoddyl::Contact3D);
  RobotEENames quadrupedAnymal("Anymal", contact_names, contact_types,
                               EXAMPLE_ROBOT_DATA_MODEL_DIR "/anymal_b_simple_description/robots/anymal.urdf",
                               EXAMPLE_ROBOT_DATA_MODEL_DIR "/anymal_b_simple_description/srdf/anymal.srdf", "RH_KFE",
                               "standing");

  print_constrained_benchmark(quadrupedAnymal);
  return 0;
}
