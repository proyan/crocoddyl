///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "python/crocoddyl/multibody/multibody.hpp"
#include "crocoddyl/multibody/data/contacts.hpp"

namespace crocoddyl {
namespace python {

void exposeDataCollectorContacts() {
  typedef pinocchio::RigidContactDataTpl<double,0> RigidContactData;
  bp::class_<DataCollectorConstraints, bp::bases<DataCollectorAbstract> >(
      "DataCollectorConstraints", "Pinocchio Contact data collector.\n\n",
      bp::init<PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) >(bp::args("self", "contacts"),
                                                        "Create contact data collection.\n\n"
                                                        ":param contacts: contacts data"))
      .add_property("contacts",
                    bp::make_getter(&DataCollectorConstraints::contacts, bp::return_value_policy<bp::return_by_value>()),
                    "contacts data");

  bp::class_<DataCollectorMultibodyConstraints, bp::bases<DataCollectorMultibody, DataCollectorConstraints> >(
      "DataCollectorMultibodyConstraints", "Data collector for multibody systems in contact.\n\n",
      bp::init<pinocchio::Data*, PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) >(
          bp::args("self", "pinocchio", "contacts"),
          "Create multibody data collection.\n\n"
          ":param pinocchio: Pinocchio data\n"
          ":param contacts: contacts data")[bp::with_custodian_and_ward<1, 2>()]);

  bp::class_<DataCollectorActMultibodyConstraints, bp::bases<DataCollectorMultibodyConstraints, DataCollectorActuation> >(
      "DataCollectorActMultibodyConstraints", "Data collector for actuated multibody systems in contact.\n\n",
      bp::init<pinocchio::Data*, boost::shared_ptr<ActuationDataAbstract>,PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) >(
          bp::args("self", "pinocchio", "actuation", "contacts"),
          "Create multibody data collection.\n\n"
          ":param pinocchio: Pinocchio data\n"
          ":param actuation: actuation data\n"
          ":param contacts: contacts data")[bp::with_custodian_and_ward<1, 2>()]);
  
  bp::class_<DataCollectorContact, bp::bases<DataCollectorAbstract> >(
      "DataCollectorContact", "Contact data collector.\n\n",
      bp::init<boost::shared_ptr<ContactDataMultiple> >(bp::args("self", "contacts"),
                                                        "Create contact data collection.\n\n"
                                                        ":param contacts: contacts data"))
      .add_property("contacts",
                    bp::make_getter(&DataCollectorContact::contacts, bp::return_value_policy<bp::return_by_value>()),
                    "contacts data");

  bp::class_<DataCollectorMultibodyInContact, bp::bases<DataCollectorMultibody, DataCollectorContact> >(
      "DataCollectorMultibodyInContact", "Data collector for multibody systems in contact.\n\n",
      bp::init<pinocchio::Data*, boost::shared_ptr<ContactDataMultiple> >(
          bp::args("self", "pinocchio", "contacts"),
          "Create multibody data collection.\n\n"
          ":param pinocchio: Pinocchio data\n"
          ":param contacts: contacts data")[bp::with_custodian_and_ward<1, 2>()]);

  bp::class_<DataCollectorActMultibodyInContact, bp::bases<DataCollectorMultibodyInContact, DataCollectorActuation> >(
      "DataCollectorActMultibodyInContact", "Data collector for actuated multibody systems in contact.\n\n",
      bp::init<pinocchio::Data*, boost::shared_ptr<ActuationDataAbstract>, boost::shared_ptr<ContactDataMultiple> >(
          bp::args("self", "pinocchio", "actuation", "contacts"),
          "Create multibody data collection.\n\n"
          ":param pinocchio: Pinocchio data\n"
          ":param actuation: actuation data\n"
          ":param contacts: contacts data")[bp::with_custodian_and_ward<1, 2>()]);
}

}  // namespace python
}  // namespace crocoddyl
