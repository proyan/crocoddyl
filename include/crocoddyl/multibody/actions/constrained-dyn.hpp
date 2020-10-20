///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MULTIBODY_ACTIONS_CONSTRAINED_DYN_HPP_
#define CROCODDYL_MULTIBODY_ACTIONS_CONSTRAINED_DYN_HPP_

#include <stdexcept>

#include <pinocchio/algorithm/contact-info.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/contact-dynamics-derivatives.hpp>

#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/data/contacts.hpp"
#include "crocoddyl/core/diff-action-base.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "crocoddyl/multibody/actuations/floating-base.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"

namespace crocoddyl {

template <typename _Scalar>
class DifferentialActionModelConstrainedDynamicsTpl : public DifferentialActionModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef DifferentialActionModelAbstractTpl<Scalar> Base;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef CostModelSumTpl<Scalar> CostModelSum;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef ActuationModelFloatingBaseTpl<Scalar> ActuationModelFloatingBase;
  typedef DifferentialActionDataAbstractTpl<Scalar> DifferentialActionDataAbstract;
  typedef DifferentialActionDataConstrainedDynamicsTpl<Scalar> DifferentialActionDataConstrainedDynamics;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef pinocchio::RigidContactModelTpl<Scalar,0> RigidContactModel;
  
  DifferentialActionModelConstrainedDynamicsTpl(boost::shared_ptr<StateMultibody> state,
                                               boost::shared_ptr<ActuationModelFloatingBase> actuation,
                                               const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel)& contacts,
                                               boost::shared_ptr<CostModelSum> costs);
  ~DifferentialActionModelConstrainedDynamicsTpl();

  virtual void calc(const boost::shared_ptr<DifferentialActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);
  virtual void calcDiff(const boost::shared_ptr<DifferentialActionDataAbstract>& data,
                        const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u);
  virtual boost::shared_ptr<DifferentialActionDataAbstract> createData();
  
  const boost::shared_ptr<ActuationModelFloatingBase>& get_actuation() const;
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel)& get_contacts() const;
  const boost::shared_ptr<CostModelSum>& get_costs() const;
  pinocchio::ModelTpl<Scalar>& get_pinocchio() const;
  const VectorXs& get_armature() const;
  const Scalar& get_damping_factor() const;

  void set_armature(const VectorXs& armature);
  void set_damping_factor(const Scalar& damping);

 protected:
  using Base::has_control_limits_;  //!< Indicates whether any of the control limits
  using Base::nr_;                  //!< Dimension of the cost residual
  using Base::nu_;                  //!< Control dimension
  using Base::state_;               //!< Model of the state
  using Base::u_lb_;                //!< Lower control limits
  using Base::u_ub_;                //!< Upper control limits
  using Base::unone_;               //!< Neutral state

 private:
  boost::shared_ptr<ActuationModelFloatingBase> actuation_;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) contacts_;
  boost::shared_ptr<CostModelSum> costs_;
  pinocchio::ModelTpl<Scalar>& pinocchio_;
  bool enable_force_;
};

template <typename _Scalar>
struct DifferentialActionDataConstrainedDynamicsTpl : public DifferentialActionDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef DifferentialActionDataAbstractTpl<Scalar> Base;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef pinocchio::RigidContactDataTpl<Scalar,0> RigidContactData;
  typedef pinocchio::RigidContactModelTpl<Scalar,0> RigidContactModel;
  
  template <template <typename Scalar> class Model>
  explicit DifferentialActionDataConstrainedDynamicsTpl(Model<Scalar>* const model)
      : Base(model),
        pinocchio(pinocchio::DataTpl<Scalar>(model->get_pinocchio())),
        multibody(&pinocchio, model->get_actuation()->createData(), PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData)() ),
        costs(model->get_costs()->createData(&multibody)) {
    costs->shareMemory(this);

    for(unsigned int i=0; i<model->get_contacts().size(); i++) {
      multibody.contacts.push_back(RigidContactData(model->get_contacts()[i]));
      multibody.frames.push_back(model->get_contacts()[i].frame_id);
    }
  }
    
  pinocchio::DataTpl<Scalar> pinocchio;
  DataCollectorActMultibodyConstraintsTpl<Scalar> multibody;
  boost::shared_ptr<CostDataSumTpl<Scalar> > costs;

  using Base::cost;
  using Base::Fu;
  using Base::Fx;
  using Base::Lu;
  using Base::Luu;
  using Base::Lx;
  using Base::Lxu;
  using Base::Lxx;
  using Base::r;
  using Base::xout;
};

}  // namespace crocoddyl

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include <crocoddyl/multibody/actions/constrained-dyn.hxx>

#endif  // CROCODDYL_MULTIBODY_ACTIONS_CONSTRAINED_DYN_HPP_

