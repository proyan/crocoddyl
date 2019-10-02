import os
import sys

import crocoddyl
import pinocchio
import numpy as np
import example_robot_data

WITHDISPLAY = 'display' in sys.argv or 'CROCODDYL_DISPLAY' in os.environ
WITHPLOT = 'plot' in sys.argv or 'CROCODDYL_PLOT' in os.environ

# In this example test, we will solve the reaching-goal task with the Talos arm.
# For that, we use the forward dynamics (with its analytical derivatives)
# developed inside crocoddyl; it describes inside DifferentialActionModelFullyActuated class.
# Finally, we use an Euler sympletic integration scheme.

# First, let's load the Pinocchio model for the Talos arm.
talos_arm = example_robot_data.loadTalosArm()
robot_model = talos_arm.model

# Create a cost model per the running and terminal action model.
state = crocoddyl.StateMultibody(robot_model)
runningCostModel = crocoddyl.CostModelSum(state)
terminalCostModel = crocoddyl.CostModelSum(state)

# Note that we need to include a cost model (i.e. set of cost functions) in
# order to fully define the action model for our optimal control problem.
# For this particular example, we formulate three running-cost functions:
# goal-tracking cost, state and control regularization; and one terminal-cost:
# goal cost. First, let's create the common cost functions.
Mref = crocoddyl.FramePlacement(robot_model.getFrameId("gripper_left_joint"),
                                pinocchio.SE3(np.eye(3), np.matrix([[.0], [.0], [.4]])))
goalTrackingCost = crocoddyl.CostModelFramePlacement(state, Mref)
xRegCost = crocoddyl.CostModelState(state)
uRegCost = crocoddyl.CostModelControl(state)

# Then let's added the running and terminal cost functions
runningCostModel.addCost("gripperPose", goalTrackingCost, 1e-3)
runningCostModel.addCost("xReg", xRegCost, 1e-7)
runningCostModel.addCost("uReg", uRegCost, 1e-7)
terminalCostModel.addCost("gripperPose", goalTrackingCost, 1)

# Next, we need to create an action model for running and terminal knots. The
# forward dynamics (computed using ABA) are implemented
# inside DifferentialActionModelFullyActuated.
dt = 1e-3
runningModel = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(state, runningCostModel), dt)
runningModel.differential.armature = np.matrix([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.]).T
terminalModel = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(state, terminalCostModel), 0.)
terminalModel.differential.armature = np.matrix([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.]).T

# For this optimal control problem, we define 250 knots (or running action
# models) plus a terminal knot
T = 250
q0 = np.matrix([0.173046, 1., -0.52366, 0., 0., 0.1, -0.005]).T
x0 = np.concatenate([q0, pinocchio.utils.zero(robot_model.nv)])
problem = crocoddyl.ShootingProblem(x0, [runningModel] * T, terminalModel)

# Creating the DDP solver for this OC problem, defining a logger
ddp = crocoddyl.SolverDDP(problem)
cameraTF = [2., 2.68, 0.54, 0.2, 0.62, 0.72, 0.22]
if WITHDISPLAY and WITHPLOT:
    ddp.setCallbacks([
        crocoddyl.CallbackLogger(),
        crocoddyl.CallbackVerbose(),
        crocoddyl.CallbackDisplay(talos_arm, 4, 4, cameraTF)
    ])
elif WITHDISPLAY:
    ddp.setCallbacks([crocoddyl.CallbackVerbose(), crocoddyl.CallbackVerbose()])
elif WITHPLOT:
    ddp.setCallbacks([
        crocoddyl.CallbackLogger(),
        crocoddyl.CallbackVerbose(),
    ])
else:
    ddp.setCallbacks([crocoddyl.CallbackVerbose()])

# Solving it with the DDP algorithm
ddp.solve()

# Plotting the solution and the DDP convergence
if WITHPLOT:
    log = ddp.getCallbacks()[0]
    crocoddyl.plotOCSolution(log.xs, log.us, figIndex=1, show=False)
    crocoddyl.plotConvergence(log.costs,
                              log.control_regs,
                              log.state_regs,
                              log.gm_stops,
                              log.th_stops,
                              log.steps,
                              figIndex=2)

# Visualizing the solution in gepetto-viewer
if WITHDISPLAY:
    crocoddyl.displayTrajectory(talos_arm, ddp.xs, runningModel.dt)