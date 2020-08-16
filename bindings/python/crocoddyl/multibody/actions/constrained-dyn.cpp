///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "python/crocoddyl/multibody/multibody.hpp"
#include "python/crocoddyl/core/diff-action-base.hpp"
#include "crocoddyl/multibody/actions/constrained-dyn.hpp"

namespace crocoddyl {
namespace python {

void exposeDifferentialActionConstrainedDynamics() {
  typedef pinocchio::RigidContactModelTpl<double,0> RigidContactModel;
  bp::class_<DifferentialActionModelConstrainedDynamics, bp::bases<DifferentialActionModelAbstract> >(
      "DifferentialActionModelConstrainedDynamics",
      "Differential action model for contact forward dynamics in multibody systems.\n\n"
      "The contact is modelled as holonomic constraits in the contact frame. There\n"
      "is also a custom implementation in case of system with armatures. If you want to\n"
      "include the armature, you need to use setArmature(). On the other hand, the\n"
      "stack of cost functions are implemented in CostModelSum().",
      bp::init<boost::shared_ptr<StateMultibody>, boost::shared_ptr<ActuationModelFloatingBase>, PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel), boost::shared_ptr<CostModelSum> >(
          bp::args("self", "state", "actuation", "contacts", "costs"),
          "Initialize the constrained forward-dynamics action model.\n\n"
          "The damping factor is needed when the contact Jacobian is not full-rank. Otherwise,\n"
          "a good damping factor could be 1e-12. In addition, if you have cost based on forces,\n"
          "you need to enable the computation of the force Jacobians (i.e. enable_force=True)."
          ":param state: multibody state\n"
          ":param actuation: floating-base actuation model\n"
          ":param contacts: multiple contact model\n"
          ":param costs: stack of cost functions\n"))
      .def<void (DifferentialActionModelConstrainedDynamics::*)(
          const boost::shared_ptr<DifferentialActionDataAbstract>&, const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &DifferentialActionModelConstrainedDynamics::calc, bp::args("self", "data", "x", "u"),
          "Compute the next state and cost value.\n\n"
          "It describes the time-continuous evolution of the multibody system with contact. The\n"
          "contacts are modelled as holonomic constraints.\n"
          "Additionally it computes the cost value associated to this state and control pair.\n"
          ":param data: contact forward-dynamics action data\n"
          ":param x: time-continuous state vector\n"
          ":param u: time-continuous control input")
      .def<void (DifferentialActionModelConstrainedDynamics::*)(
          const boost::shared_ptr<DifferentialActionDataAbstract>&, const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calc", &DifferentialActionModelAbstract::calc, bp::args("self", "data", "x"))
      .def<void (DifferentialActionModelConstrainedDynamics::*)(
          const boost::shared_ptr<DifferentialActionDataAbstract>&, const Eigen::Ref<const Eigen::VectorXd>&,
          const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &DifferentialActionModelConstrainedDynamics::calcDiff, bp::args("self", "data", "x", "u"),
          "Compute the derivatives of the differential multibody system and its cost\n"
          "functions.\n\n"
          "It computes the partial derivatives of the differential multibody system and the\n"
          "cost function. It assumes that calc has been run first.\n"
          "This function builds a quadratic approximation of the\n"
          "action model (i.e. dynamical system and cost function).\n"
          ":param data: contact forward-dynamics action data\n"
          ":param x: time-continuous state vector\n"
          ":param u: time-continuous control input\n")
      .def<void (DifferentialActionModelConstrainedDynamics::*)(
          const boost::shared_ptr<DifferentialActionDataAbstract>&, const Eigen::Ref<const Eigen::VectorXd>&)>(
          "calcDiff", &DifferentialActionModelAbstract::calcDiff, bp::args("self", "data", "x"))
      .def("createData", &DifferentialActionModelConstrainedDynamics::createData, bp::args("self"),
           "Create the contact forward dynamics differential action data.")
      .add_property("pinocchio",
                    bp::make_function(&DifferentialActionModelConstrainedDynamics::get_pinocchio,
                                      bp::return_internal_reference<>()),
                    "multibody model (i.e. pinocchio model)")
      .add_property("actuation",
                    bp::make_function(&DifferentialActionModelConstrainedDynamics::get_actuation,
                                      bp::return_value_policy<bp::return_by_value>()),
                    "actuation model")
      .add_property("contacts",
                    bp::make_function(&DifferentialActionModelConstrainedDynamics::get_contacts,
                                      bp::return_value_policy<bp::return_by_value>()),
                    "multiple contact model")
      .add_property("costs",
                    bp::make_function(&DifferentialActionModelConstrainedDynamics::get_costs,
                                      bp::return_value_policy<bp::return_by_value>()),
                    "total cost model")
      .add_property("armature",
                    bp::make_function(&DifferentialActionModelConstrainedDynamics::get_armature,
                                      bp::return_value_policy<bp::return_by_value>()),
                    bp::make_function(&DifferentialActionModelConstrainedDynamics::set_armature),
                    "set an armature mechanism in the joints");

  bp::register_ptr_to_python<boost::shared_ptr<DifferentialActionDataConstrainedDynamics> >();

  bp::class_<DifferentialActionDataConstrainedDynamics, bp::bases<DifferentialActionDataAbstract> >(
      "DifferentialActionDataConstrainedDynamics", "Action data for the contact forward dynamics system.",
      bp::init<DifferentialActionModelConstrainedDynamics*>(bp::args("self", "model"),
                                                           "Create contact forward-dynamics action data.\n\n"
                                                           ":param model: contact forward-dynamics action model"))
      .add_property(
          "pinocchio",
          bp::make_getter(&DifferentialActionDataConstrainedDynamics::pinocchio, bp::return_internal_reference<>()),
          "pinocchio data")
      .add_property(
          "multibody",
          bp::make_getter(&DifferentialActionDataConstrainedDynamics::multibody, bp::return_internal_reference<>()),
          "multibody data")
      .add_property("costs",
                    bp::make_getter(&DifferentialActionDataConstrainedDynamics::costs,
                                    bp::return_value_policy<bp::return_by_value>()),
                    "total cost data");
}

}  // namespace python
}  // namespace crocoddyl

