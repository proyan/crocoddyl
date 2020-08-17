///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifdef CROCODDYL_WITH_MULTITHREADING
#include <omp.h>
#define NUM_THREADS CROCODDYL_WITH_NTHREADS
#else
#define NUM_THREADS 1
#endif

#ifdef CROCODDYL_WITH_CODEGEN
#include "crocoddyl/core/codegen/action-base.hpp"
#endif

#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/utils/timer.hpp"
#include "crocoddyl/core/utils/file-io.hpp"

#include "factory/legged-constrained-robots.hpp"
#include "factory/arm.hpp"
#include "factory/arm-kinova.hpp"

#define STDDEV(vec) std::sqrt(((vec - vec.mean())).square().sum() / (static_cast<double>(vec.size()) - 1))
#define AVG(vec) (vec.mean())

void print_constrained_benchmark(RobotEENames robot) {
  unsigned int N = 100;  // number of nodes
  unsigned int T = 1e3;  // number of trials

  // Building the running and terminal models
  boost::shared_ptr<crocoddyl::ActionModelAbstract> runningModel, terminalModel;
  if (robot.robot_name == "Talos_arm") {
    crocoddyl::benchmark::build_arm_action_models(runningModel, terminalModel);
  } else if (robot.robot_name == "Kinova_arm") {
    crocoddyl::benchmark::build_arm_kinova_action_models(runningModel, terminalModel);
  } else {
    crocoddyl::benchmark::build_constrained_action_models(robot, runningModel, terminalModel);
  }

  // Get the initial state
  boost::shared_ptr<crocoddyl::StateMultibody> state =
      boost::static_pointer_cast<crocoddyl::StateMultibody>(runningModel->get_state());
  std::cout << "NQ: " << state->get_nq() << std::endl;
  std::cout << "Number of nodes: " << N << std::endl << std::endl;

  Eigen::VectorXd default_state(state->get_nq() + state->get_nv());
  boost::shared_ptr<crocoddyl::IntegratedActionModelEulerTpl<double> > rm =
      boost::static_pointer_cast<crocoddyl::IntegratedActionModelEulerTpl<double> >(runningModel);
  if (robot.robot_name == "Talos_arm" || robot.robot_name == "Kinova_arm") {
    boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamicsTpl<double> > dm =
        boost::static_pointer_cast<crocoddyl::DifferentialActionModelFreeFwdDynamicsTpl<double> >(
            rm->get_differential());
    default_state << dm->get_pinocchio().referenceConfigurations[robot.reference_conf],
        Eigen::VectorXd::Zero(state->get_nv());
  } else {
    boost::shared_ptr<crocoddyl::DifferentialActionModelConstrainedDynamicsTpl<double> > dm =
        boost::static_pointer_cast<crocoddyl::DifferentialActionModelConstrainedDynamicsTpl<double> >(
            rm->get_differential());
    default_state << dm->get_pinocchio().referenceConfigurations[robot.reference_conf],
        Eigen::VectorXd::Zero(state->get_nv());
  }
  Eigen::VectorXd x0(default_state);
  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract> > runningModels(N, runningModel);
  boost::shared_ptr<crocoddyl::ShootingProblem> problem =
      boost::make_shared<crocoddyl::ShootingProblem>(x0, runningModels, terminalModel);
  // Computing the warm-start
  std::vector<Eigen::VectorXd> xs(N + 1, x0);
  std::vector<Eigen::VectorXd> us(N, Eigen::VectorXd::Zero(runningModel->get_nu()));
  for (unsigned int i = 0; i < N; ++i) {
    const boost::shared_ptr<crocoddyl::ActionModelAbstract>& model = problem->get_runningModels()[i];
    const boost::shared_ptr<crocoddyl::ActionDataAbstract>& data = problem->get_runningDatas()[i];
    model->quasiStatic(data, us[i], x0);
  }
  crocoddyl::SolverFDDP ddp(problem);
  ddp.setCandidate(xs, us, false);
  boost::shared_ptr<crocoddyl::ActionDataAbstract> runningData = runningModel->createData();

#ifdef CROCODDYL_WITH_CODEGEN
  // Code generation of the running an terminal models
  typedef CppAD::AD<CppAD::cg::CG<double> > ADScalar;
  boost::shared_ptr<crocoddyl::ActionModelAbstractTpl<ADScalar> > ad_runningModel, ad_terminalModel;
  if (robot.robot_name == "Talos_arm") {
    crocoddyl::benchmark::build_arm_action_models(ad_runningModel, ad_terminalModel);
  } else if (robot.robot_name == "Kinova_arm") {
    crocoddyl::benchmark::build_arm_kinova_action_models(ad_runningModel, ad_terminalModel);
  } else {
    crocoddyl::benchmark::build_constrained_action_models(robot, ad_runningModel, ad_terminalModel);
  }

  boost::shared_ptr<crocoddyl::ActionModelAbstract> cg_runningModel =
      boost::make_shared<crocoddyl::ActionModelCodeGen>(ad_runningModel, runningModel,
                                                        robot.robot_name + "_running_cg");
  boost::shared_ptr<crocoddyl::ActionModelAbstract> cg_terminalModel =
      boost::make_shared<crocoddyl::ActionModelCodeGen>(ad_terminalModel, terminalModel,
                                                        robot.robot_name + "_terminal_cg");

  // Defining the shooting problem for both cases: with and without code generation
  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract> > cg_runningModels(N, cg_runningModel);
  boost::shared_ptr<crocoddyl::ShootingProblem> cg_problem =
      boost::make_shared<crocoddyl::ShootingProblem>(x0, cg_runningModels, cg_terminalModel);

  // Check that code-generated action model is the same as original.
  /**************************************************************************/
  crocoddyl::SolverFDDP cg_ddp(cg_problem);
  cg_ddp.setCandidate(xs, us, false);
  boost::shared_ptr<crocoddyl::ActionDataAbstract> cg_runningData = cg_runningModel->createData();
  Eigen::VectorXd x_rand = cg_runningModel->get_state()->rand();
  Eigen::VectorXd u_rand = Eigen::VectorXd::Random(cg_runningModel->get_nu());
  runningModel->calc(runningData, x_rand, u_rand);
  runningModel->calcDiff(runningData, x_rand, u_rand);
  cg_runningModel->calc(cg_runningData, x_rand, u_rand);
  cg_runningModel->calcDiff(cg_runningData, x_rand, u_rand);
  assert_pretty(cg_runningData->xnext.isApprox(runningData->xnext), "Problem in xnext");
  assert_pretty(std::abs(cg_runningData->cost - runningData->cost) < 1e-10, "Problem in cost");
  assert_pretty(cg_runningData->Lx.isApprox(runningData->Lx), "Problem in Lx");
  assert_pretty(cg_runningData->Lu.isApprox(runningData->Lu), "Problem in Lu");
  assert_pretty(cg_runningData->Lxx.isApprox(runningData->Lxx), "Problem in Lxx");
  assert_pretty(cg_runningData->Lxu.isApprox(runningData->Lxu), "Problem in Lxu");
  assert_pretty(cg_runningData->Luu.isApprox(runningData->Luu), "Problem in Luu");
  assert_pretty(cg_runningData->Fx.isApprox(runningData->Fx), "Problem in Fx");
  assert_pretty(cg_runningData->Fu.isApprox(runningData->Fu), "Problem in Fu");
#endif  // CROCODDYL_WITH_CODEGEN

  /******************************* create csv file *******************************/
  const std::string csv_filename = "/tmp/" + robot.robot_name + "_" + std::to_string(state->get_nq()) + "DoF_constrained.bench";
  CsvStream csv(csv_filename);
  csv << "fn_name"
      << "nthreads"
      << "with_cg"
      << "mean"
      << "stddev"
      << "max"
      << "min"
      << "mean_per_nodes"
      << "stddev_per_nodes" << csv.endl;

  /*******************************************************************************/
  /*********************************** TIMINGS ***********************************/
  Eigen::ArrayXd duration(T);
  Eigen::ArrayXd avg(NUM_THREADS);
  Eigen::ArrayXd stddev(NUM_THREADS);

  /*******************************************************************************/
  /****************************** ACTION MODEL TIMINGS ***************************/
  std::cout << "Without Code Generation:" << std::endl;
  problem->calc(xs, us);
  // calcDiff timings
  for (int ithread = 0; ithread < NUM_THREADS; ++ithread) {
    duration.setZero();
#ifdef CROCODDYL_WITH_MULTITHREADING
    omp_set_num_threads(ithread + 1);
#endif
    for (unsigned int i = 0; i < T; ++i) {
      crocoddyl::Timer timer;
#ifdef CROCODDYL_WITH_MULTITHREADING
#pragma omp parallel for
#endif
      for (unsigned int j = 0; j < N; ++j) {
        runningModels[j]->calcDiff(problem->get_runningDatas()[j], xs[j], us[j]);
      }
      duration[i] = timer.get_us_duration();
    }
    avg[ithread] = AVG(duration);
    stddev[ithread] = STDDEV(duration);
    std::cout << ithread + 1 << " threaded calcDiff [us]:\t" << avg[ithread] << " +- " << stddev[ithread]
              << " (max: " << duration.maxCoeff() << ", min: " << duration.minCoeff()
              << ", per nodes: " << avg[ithread] * (ithread + 1) / N << " +- " << stddev[ithread] * (ithread + 1) / N
              << ")" << std::endl;
    csv << "calcDiff" << (ithread + 1) << false << avg[ithread] << stddev[ithread] << duration.maxCoeff()
        << duration.minCoeff() << avg[ithread] * (ithread + 1) / N << stddev[ithread] * (ithread + 1) / N << csv.endl;
  }

  // calc timings
  for (int ithread = 0; ithread < NUM_THREADS; ++ithread) {
    duration.setZero();
#ifdef CROCODDYL_WITH_MULTITHREADING
    omp_set_num_threads(ithread + 1);
#endif
    for (unsigned int i = 0; i < T; ++i) {
      crocoddyl::Timer timer;
#ifdef CROCODDYL_WITH_MULTITHREADING
#pragma omp parallel for
#endif
      for (unsigned int j = 0; j < N; ++j) {
        runningModels[j]->calc(problem->get_runningDatas()[j], xs[j], us[j]);
      }
      duration[i] = timer.get_us_duration();
    }
    avg[ithread] = AVG(duration);
    stddev[ithread] = STDDEV(duration);
    std::cout << ithread + 1 << " threaded calc [us]:    \t" << avg[ithread] << " +- " << stddev[ithread]
              << " (max: " << duration.maxCoeff() << ", min: " << duration.minCoeff()
              << ", per nodes: " << avg[ithread] * (ithread + 1) / N << " +- " << stddev[ithread] * (ithread + 1) / N
              << ")" << std::endl;
    csv << "calc" << (ithread + 1) << false << avg[ithread] << stddev[ithread] << duration.maxCoeff()
        << duration.minCoeff() << avg[ithread] * (ithread + 1) / N << stddev[ithread] * (ithread + 1) / N << csv.endl;
  }

  /*******************************************************************************/
  /******************* DDP BACKWARD AND FORWARD PASSES TIMINGS *******************/
  // Backward pass timings
  ddp.calcDiff();
  duration.setZero();
  for (unsigned int i = 0; i < T; ++i) {
    crocoddyl::Timer timer;
    ddp.backwardPass();
    duration[i] = timer.get_us_duration();
  }
  double avg_bp = AVG(duration);
  double stddev_bp = STDDEV(duration);
  std::cout << "backwardPass [us]:\t\t" << avg_bp << " +- " << stddev_bp << " (max: " << duration.maxCoeff()
            << ", min: " << duration.minCoeff() << ", per nodes: " << avg_bp / N << " +- " << stddev_bp / N << ")"
            << std::endl;

  csv << "backwardPass" << 1 << false << avg_bp << stddev_bp << duration.maxCoeff() << duration.minCoeff()
      << avg_bp / N << stddev_bp / N << csv.endl;

  // Forward pass timings
  duration.setZero();
  for (unsigned int i = 0; i < T; ++i) {
    crocoddyl::Timer timer;
    ddp.forwardPass(0.005);
    duration[i] = timer.get_us_duration();
  }
  double avg_fp = AVG(duration);
  double stddev_fp = STDDEV(duration);
  std::cout << "forwardPass [us]: \t\t" << avg_fp << " +- " << stddev_fp << " (max: " << duration.maxCoeff()
            << ", min: " << duration.minCoeff() << ", per nodes: " << avg_fp / N << " +- " << stddev_fp / N << ")"
            << std::endl;

  csv << "forwardPass" << 1 << false << avg_fp << stddev_fp << duration.maxCoeff() << duration.minCoeff() << avg_fp / N
      << stddev_fp / N << csv.endl;

#ifdef CROCODDYL_WITH_CODEGEN

  /*******************************************************************************/
  /*************************** CODE GENERATION TIMINGS ***************************/
  /*******************************************************************************/

  /*******************************************************************************/
  /****************************** ACTION MODEL TIMINGS ***************************/
  std::cout << std::endl << "With Code Generation:" << std::endl;
  // calcDiff timings
  cg_problem->calc(xs, us);
  for (int ithread = 0; ithread < NUM_THREADS; ++ithread) {
    duration.setZero();
#ifdef CROCODDYL_WITH_MULTITHREADING
    omp_set_num_threads(ithread + 1);
#endif
    for (unsigned int i = 0; i < T; ++i) {
      crocoddyl::Timer timer;
#ifdef CROCODDYL_WITH_MULTITHREADING
#pragma omp parallel for
#endif
      for (unsigned int j = 0; j < N; ++j) {
        cg_runningModels[j]->calcDiff(cg_problem->get_runningDatas()[j], xs[j], us[j]);
      }
      duration[i] = timer.get_us_duration();
    }
    avg[ithread] = AVG(duration);
    stddev[ithread] = STDDEV(duration);
    std::cout << ithread + 1 << " threaded calcDiff [us]:\t" << avg[ithread] << " +- " << stddev[ithread]
              << " (max: " << duration.maxCoeff() << ", min: " << duration.minCoeff()
              << ", per nodes: " << avg[ithread] * (ithread + 1) / N << " +- " << stddev[ithread] * (ithread + 1) / N
              << ")" << std::endl;
    csv << "calcDiff" << (ithread + 1) << true << avg[ithread] << stddev[ithread] << duration.maxCoeff()
        << duration.minCoeff() << avg[ithread] * (ithread + 1) / N << stddev[ithread] * (ithread + 1) / N << csv.endl;
  }

  // calc timings
  for (int ithread = 0; ithread < NUM_THREADS; ++ithread) {
    duration.setZero();
#ifdef CROCODDYL_WITH_MULTITHREADING
    omp_set_num_threads(ithread + 1);
#endif
    for (unsigned int i = 0; i < T; ++i) {
      crocoddyl::Timer timer;
#ifdef CROCODDYL_WITH_MULTITHREADING
#pragma omp parallel for
#endif
      for (unsigned int j = 0; j < N; ++j) {
        cg_runningModels[j]->calc(cg_problem->get_runningDatas()[j], xs[j], us[j]);
      }
      duration[i] = timer.get_us_duration();
    }
    avg[ithread] = AVG(duration);
    stddev[ithread] = STDDEV(duration);
    std::cout << ithread + 1 << " threaded calc [us]:    \t" << avg[ithread] << " +- " << stddev[ithread]
              << " (max: " << duration.maxCoeff() << ", min: " << duration.minCoeff()
              << ", per nodes: " << avg[ithread] * (ithread + 1) / N << " +- " << stddev[ithread] * (ithread + 1) / N
              << ")" << std::endl;
    csv << "calc" << (ithread + 1) << true << avg[ithread] << stddev[ithread] << duration.maxCoeff()
        << duration.minCoeff() << avg[ithread] * (ithread + 1) / N << stddev[ithread] * (ithread + 1) / N << csv.endl;
  }

  /*******************************************************************************/
  /******************* DDP BACKWARD AND FORWARD PASSES TIMINGS *******************/
  // Backward pass timings
  cg_ddp.calcDiff();
  for (unsigned int i = 0; i < T; ++i) {
    crocoddyl::Timer timer;
    cg_ddp.backwardPass();
    duration[i] = timer.get_us_duration();
  }
  avg_bp = AVG(duration);
  stddev_bp = STDDEV(duration);
  std::cout << "backwardPass [us]:\t\t" << avg_bp << " +- " << stddev_bp << " (max: " << duration.maxCoeff()
            << ", min: " << duration.minCoeff() << ", per nodes: " << avg_bp / N << " +- " << stddev_bp / N << ")"
            << std::endl;
  csv << "backwardPass" << 1 << true << avg_bp << stddev_bp << duration.maxCoeff() << duration.minCoeff() << avg_bp / N
      << stddev_bp / N << csv.endl;

  // Forward pass timings
  for (unsigned int i = 0; i < T; ++i) {
    crocoddyl::Timer timer;
    cg_ddp.forwardPass(0.005);
    duration[i] = timer.get_us_duration();
  }
  avg_fp = AVG(duration);
  stddev_fp = STDDEV(duration);
  std::cout << "forwardPass [us]: \t\t" << avg_fp << " +- " << stddev_fp << " (max: " << duration.maxCoeff()
            << ", min: " << duration.minCoeff() << ", per nodes: " << avg_fp / N << " +- " << stddev_fp / N << ")"
            << std::endl;

  csv << "forwardPass" << 1 << true << avg_fp << stddev_fp << duration.maxCoeff() << duration.minCoeff() << avg_fp / N
      << stddev_fp / N << csv.endl;

#endif  // CROCODDYL_WITH_CODEGEN
}
