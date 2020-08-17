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
  std::vector<std::string> contact_names, locked_joints;
  std::vector<crocoddyl::ContactType> contact_types;

  // Biped icub Benchmarks
  std::cout << "********************Biped Talos***********************" << std::endl;
  contact_names.clear();
  contact_types.clear();
  contact_names.push_back("leg_right_6_joint");
  contact_names.push_back("leg_left_6_joint");
  contact_types.push_back(crocoddyl::Contact6D);
  contact_types.push_back(crocoddyl::Contact6D);

  locked_joints.push_back("arm_left_5_joint");
  locked_joints.push_back("arm_left_6_joint");
  locked_joints.push_back("arm_left_7_joint");
  locked_joints.push_back("arm_right_5_joint");
  locked_joints.push_back("arm_right_6_joint");
  locked_joints.push_back("arm_right_7_joint");
  locked_joints.push_back("gripper_left_joint");
  locked_joints.push_back("gripper_right_joint");
  locked_joints.push_back("head_1_joint");
  locked_joints.push_back("head_2_joint");
  
  RobotEENames bipedTalos(
      "Talos", contact_names, contact_types, EXAMPLE_ROBOT_DATA_MODEL_DIR "/talos_data/robots/talos_reduced.urdf",
      EXAMPLE_ROBOT_DATA_MODEL_DIR "/talos_data/srdf/talos.srdf", "arm_right_7_joint", "half_sitting", locked_joints);
  print_benchmark(bipedTalos);
  return 0;
}
