///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/multibody/actions/constrained-dyn.hpp"

#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>

namespace crocoddyl {

template <typename Scalar>
DifferentialActionModelConstrainedDynamicsTpl<Scalar>::DifferentialActionModelConstrainedDynamicsTpl(
    boost::shared_ptr<StateMultibody> state,
    boost::shared_ptr<ActuationModelFloatingBase> actuation,
    const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel)& contacts,
    boost::shared_ptr<CostModelSum> costs)
    : Base(state, actuation->get_nu(), costs->get_nr()),
      actuation_(actuation),
      contacts_(contacts),
      costs_(costs),
      pinocchio_(*state->get_pinocchio().get()) {
  if (costs_->get_nu() != nu_) {
    throw_pretty("Invalid argument: "
                 << "Costs doesn't have the same control dimension (it should be " + std::to_string(nu_) + ")");
  }

  Base::set_u_lb(Scalar(-1.) * pinocchio_.effortLimit.tail(nu_));
  Base::set_u_ub(Scalar(+1.) * pinocchio_.effortLimit.tail(nu_));
}

template <typename Scalar>
DifferentialActionModelConstrainedDynamicsTpl<Scalar>::~DifferentialActionModelConstrainedDynamicsTpl() {}

template <typename Scalar>
void DifferentialActionModelConstrainedDynamicsTpl<Scalar>::calc(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
    const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(nu_) + ")");
  }

  DifferentialActionDataConstrainedDynamicsTpl<Scalar>* d =
      static_cast<DifferentialActionDataConstrainedDynamicsTpl<Scalar>*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(state_->get_nv());
  
  actuation_->calc(d->multibody.actuation, x, u);
  d->xout = pinocchio::contactDynamics(pinocchio_, d->pinocchio, q, v,
                                       d->multibody.actuation->tau, contacts_,
                                       d->multibody.contacts);

  // Computing the cost value and residuals
  costs_->calc(d->costs, x, u);
  d->cost = d->costs->cost;
}

template <typename Scalar>
void DifferentialActionModelConstrainedDynamicsTpl<Scalar>::calcDiff(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
    const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(nu_) + ")");
  }

  const std::size_t& nv = state_->get_nv();
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.tail(nv);

  DifferentialActionDataConstrainedDynamicsTpl<Scalar>* d =
      static_cast<DifferentialActionDataConstrainedDynamicsTpl<Scalar>*>(data.get());

  // Computing the dynamics derivatives
  actuation_->calcDiff(d->multibody.actuation, x, u);
  pinocchio::computeContactDynamicsDerivatives(pinocchio_, d->pinocchio, contacts_,
                                               d->multibody.contacts, Scalar(0.),
                                               d->Fx.leftCols(nv),
                                               d->Fx.rightCols(nv),
                                               d->pinocchio.ddq_dtau,
                                               d->pinocchio.dlambda_dq,
                                               d->pinocchio.dlambda_dv,
                                               d->pinocchio.dlambda_dtau);

  d->Fu.noalias() = d->pinocchio.ddq_dtau * d->multibody.actuation->dtau_du;
  costs_->calcDiff(d->costs, x, u);
}

template <typename Scalar>
boost::shared_ptr<DifferentialActionDataAbstractTpl<Scalar> >
DifferentialActionModelConstrainedDynamicsTpl<Scalar>::createData() {

  boost::shared_ptr<DifferentialActionDataAbstractTpl<Scalar> > data =
    boost::make_shared<DifferentialActionDataConstrainedDynamics>(this);
  DifferentialActionDataConstrainedDynamics* d = 
    static_cast<DifferentialActionDataConstrainedDynamics*>(data.get());
  pinocchio::initContactDynamics(pinocchio_, d->pinocchio, contacts_);
  
  return data;
}
  
template <typename Scalar>
pinocchio::ModelTpl<Scalar>& DifferentialActionModelConstrainedDynamicsTpl<Scalar>::get_pinocchio() const {
  return pinocchio_;
}

template <typename Scalar>
const boost::shared_ptr<ActuationModelFloatingBaseTpl<Scalar> >&
DifferentialActionModelConstrainedDynamicsTpl<Scalar>::get_actuation() const {
  return actuation_;
}

template <typename Scalar>
const std::vector<pinocchio::RigidContactModelTpl<Scalar,0>,Eigen::aligned_allocator<pinocchio::RigidContactModelTpl<Scalar,0> > >&
DifferentialActionModelConstrainedDynamicsTpl<Scalar>::get_contacts() const {
  return contacts_;
}

template <typename Scalar>
const boost::shared_ptr<CostModelSumTpl<Scalar> >& DifferentialActionModelConstrainedDynamicsTpl<Scalar>::get_costs()
    const {
  return costs_;
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::VectorXs& DifferentialActionModelConstrainedDynamicsTpl<Scalar>::get_armature()
    const {
  return pinocchio_.armature;
}


template <typename Scalar>
void DifferentialActionModelConstrainedDynamicsTpl<Scalar>::set_armature(const VectorXs& armature) {
  if (static_cast<std::size_t>(armature.size()) != state_->get_nv()) {
    throw_pretty("Invalid argument: "
                 << "The armature dimension is wrong (it should be " + std::to_string(state_->get_nv()) + ")");
  }
  pinocchio_.armature = armature;
}

}  // namespace crocoddyl
