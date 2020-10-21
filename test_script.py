#!/usr/bin/env python3
import os
import sys

import numpy as np
import time
import crocoddyl
import example_robot_data
import pinocchio
from crocoddyl.utils.biped import SimpleBipedGaitProblem, plotSolution

# Creating the lower-body part of Talos
talos_legs = example_robot_data.loadTalosLegs()
# Defining the initial state of the robot
q0 = talos_legs.model.referenceConfigurations['half_sitting'].copy()
v0 = pinocchio.utils.zero(talos_legs.model.nv)
x0 = np.concatenate([q0, v0])

# Setting up the 3d walking problem
rightFoot = 'right_sole_link'
leftFoot = 'left_sole_link'
gait = SimpleBipedGaitProblem(talos_legs.model, rightFoot, leftFoot)

# Setting up all tasks
GAITPHASES = \
stepLength = 0.6
stepHeight = 0.1
timeStep = 0.03
stepKnots = 40
supportKnots = 9
ddp = crocoddyl.SolverFDDP(
  gait.createWalkingProblem(x0, stepLength, stepHeight, timeStep,
                            stepKnots, supportKnots))
print ("No of Nodes: ", len(ddp.problem.runningModels.tolist()))
# Added the callback functions
# Solving the problem with the DDP solver
total_time = 0.
N = 10000
for m in range(N):
  ddp = crocoddyl.SolverFDDP(
    gait.createWalkingProblem(x0, stepLength, stepHeight, timeStep,
                              stepKnots, supportKnots))
  ddp.th_stop = 1e-7
  xs = [talos_legs.model.defaultState] * (ddp.problem.T + 1)
  us = ddp.problem.quasiStatic([talos_legs.model.defaultState] * ddp.problem.T)
  start_time = time.perf_counter()
  ddp.solve(xs, us, 1, False, 0.1)
  end_time = time.perf_counter()
  total_time += end_time - start_time
  del ddp, xs, us
print ("time in python in ms:", total_time*1000. / N)

  
