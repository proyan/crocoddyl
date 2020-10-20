import timeit

print(timeit.timeit(stmt='ddp.solve([],[],1)',
                    setup='import crocoddyl, numpy; model = crocoddyl.ActionModelUnicycle();  problem = crocoddyl.ShootingProblem(numpy.array([-2., -2., 1.57079633]), [model] * 1000, model);  ddp = crocoddyl.SolverFDDP(problem);',
                    number=10000))
