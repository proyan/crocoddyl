import timeit
import time
print(1000000.*timeit.timeit(stmt='ddp.problem.calcDiff(xs, us)',
                             setup='import crocoddyl, numpy; from copy import copy; model = crocoddyl.ActionModelUnicycle();  problem = crocoddyl.ShootingProblem(numpy.array([-2., -2., 1.57079633]), [model] * 100000, model);  ddp = crocoddyl.SolverFDDP(problem); ddp.solve([],[],1); print(problem); xs = copy(ddp.xs); us = copy(ddp.us)', timer=time.clock, number=1))
