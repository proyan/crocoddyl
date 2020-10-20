///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_LEGGED_CONSTRAINED_ROBOTS_FACTORY_HPP_
#define CROCODDYL_LEGGED_CONSTRAINED_ROBOTS_FACTORY_HPP_

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <example-robot-data/path.hpp>

#include "robot-ee-names.hpp"
#include "crocoddyl/core/mathbase.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "crocoddyl/multibody/actuations/floating-base.hpp"
#include "crocoddyl/multibody/actions/constrained-dyn.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/multibody/costs/frame-placement.hpp"
#include "crocoddyl/multibody/costs/frame-translation.hpp"
#include "crocoddyl/multibody/costs/com-position.hpp"
#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/core/costs/control.hpp"

namespace crocoddyl {
namespace benchmark {

template <typename Scalar>
void build_constrained_action_models(RobotEENames robotNames,
                                 boost::shared_ptr<crocoddyl::ActionModelAbstractTpl<Scalar> >& runningModel,
                                 boost::shared_ptr<crocoddyl::ActionModelAbstractTpl<Scalar> >& terminalModel) {
  typedef typename crocoddyl::MathBaseTpl<Scalar>::Vector3s Vector3s;
  typedef typename crocoddyl::MathBaseTpl<Scalar>::VectorXs VectorXs;
  typedef typename crocoddyl::MathBaseTpl<Scalar>::Matrix3s Matrix3s;
  typedef typename crocoddyl::FramePlacementTpl<Scalar> FramePlacement;
  typedef typename crocoddyl::DifferentialActionModelConstrainedDynamicsTpl<Scalar>
      DifferentialActionModelConstrainedDynamics;
  typedef typename crocoddyl::IntegratedActionModelEulerTpl<Scalar> IntegratedActionModelEuler;
  typedef typename crocoddyl::ActuationModelFloatingBaseTpl<Scalar> ActuationModelFloatingBase;
  typedef typename crocoddyl::CostModelSumTpl<Scalar> CostModelSum;
  typedef typename crocoddyl::CostModelAbstractTpl<Scalar> CostModelAbstract;
  typedef typename crocoddyl::CostModelFramePlacementTpl<Scalar> CostModelFramePlacement;
  typedef typename crocoddyl::CostModelCoMPositionTpl<Scalar> CostModelCoMPosition;
  typedef typename crocoddyl::CostModelStateTpl<Scalar> CostModelState;
  typedef typename crocoddyl::CostModelControlTpl<Scalar> CostModelControl;

  pinocchio::ModelTpl<double> modeld;
  pinocchio::urdf::buildModel(robotNames.urdf_path, pinocchio::JointModelFreeFlyer(), modeld);
  modeld.lowerPositionLimit.head<7>().array() = -1;
  modeld.upperPositionLimit.head<7>().array() = 1.;
  pinocchio::srdf::loadReferenceConfigurations(modeld, robotNames.srdf_path, false);


  //Load model and fix joints.
  pinocchio::ModelTpl<Scalar> model_full(modeld.cast<Scalar>()), model;

  std::vector<pinocchio::JointIndex> locked_joints;
  for (std::size_t i = 0; i < robotNames.locked_joints.size(); ++i) {
    locked_joints.push_back(model_full.getJointId(robotNames.locked_joints[i]));
  }
  pinocchio::buildReducedModel(model_full, locked_joints,
                               pinocchio::neutral(model_full), model);

  
  boost::shared_ptr<crocoddyl::StateMultibodyTpl<Scalar> > state =
      boost::make_shared<crocoddyl::StateMultibodyTpl<Scalar> >(
          boost::make_shared<pinocchio::ModelTpl<Scalar> >(model));

  VectorXs default_state(model.nq + model.nv);
  default_state << model.referenceConfigurations[robotNames.reference_conf], VectorXs::Zero(model.nv);

  boost::shared_ptr<ActuationModelFloatingBase> actuation = boost::make_shared<ActuationModelFloatingBase>(state);

  FramePlacement Mref(model.getFrameId(robotNames.ee_name),
                      pinocchio::SE3Tpl<Scalar>(Matrix3s::Identity(), Vector3s(Scalar(.0), Scalar(.0), Scalar(.4))));

  boost::shared_ptr<CostModelAbstract> comCost =
      boost::make_shared<CostModelCoMPosition>(state, Vector3s::Zero(), actuation->get_nu());
  boost::shared_ptr<CostModelAbstract> goalTrackingCost =
      boost::make_shared<CostModelFramePlacement>(state, Mref, actuation->get_nu());
  boost::shared_ptr<CostModelAbstract> xRegCost =
      boost::make_shared<CostModelState>(state, default_state, actuation->get_nu());
  boost::shared_ptr<CostModelAbstract> uRegCost = boost::make_shared<CostModelControl>(state, actuation->get_nu());

  // Create a cost model per the running and terminal action model.
  boost::shared_ptr<CostModelSum> runningCostModel = boost::make_shared<CostModelSum>(state, actuation->get_nu());
  boost::shared_ptr<CostModelSum> terminalCostModel = boost::make_shared<CostModelSum>(state, actuation->get_nu());

  // Then let's added the running and terminal cost functions
  runningCostModel->addCost("gripperPose", goalTrackingCost, Scalar(1));
  //   runningCostModel->addCost("comPos", comCost, Scalar(1e-7));
  runningCostModel->addCost("xReg", xRegCost, Scalar(1e-4));
  runningCostModel->addCost("uReg", uRegCost, Scalar(1e-4));
  terminalCostModel->addCost("gripperPose", goalTrackingCost, Scalar(1));

  typedef pinocchio::RigidContactModelTpl<Scalar,0> RigidContactModel;
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contact_models;

  for (std::size_t i = 0; i < robotNames.contact_names.size(); ++i) {
    switch (robotNames.contact_types[i]) {
      case Contact3D: {
        RigidContactModel support_contact(pinocchio::CONTACT_3D,model.getFrameId(robotNames.contact_names[i]),pinocchio::LOCAL);
        contact_models.push_back(support_contact);
        break;
      }
      case Contact6D: {
        RigidContactModel support_contact(pinocchio::CONTACT_6D,model.getFrameId(robotNames.contact_names[i]),pinocchio::LOCAL);
        contact_models.push_back(support_contact);
        break;
      }
      default: { break; }
    }
  }

  // Next, we need to create an action model for running and terminal nodes
  boost::shared_ptr<DifferentialActionModelConstrainedDynamics> runningDAM =
      boost::make_shared<DifferentialActionModelConstrainedDynamics>(state, actuation, contact_models,
                                                                    runningCostModel);
  boost::shared_ptr<DifferentialActionModelConstrainedDynamics> terminalDAM =
      boost::make_shared<DifferentialActionModelConstrainedDynamics>(state, actuation, contact_models,
                                                                    terminalCostModel);

  runningModel = boost::make_shared<IntegratedActionModelEuler>(runningDAM, Scalar(5e-3));
  terminalModel = boost::make_shared<IntegratedActionModelEuler>(terminalDAM, Scalar(5e-3));
}

}  // namespace benchmark
}  // namespace crocoddyl

#endif  // CROCODDYL_BIPED_FACTORY_HPP_
