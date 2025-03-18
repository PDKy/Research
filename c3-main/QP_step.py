from Define_valuable import *
import numpy as np

from pydrake.all import (
    DiagramBuilder, Simulator, FindResourceOrThrow, MultibodyPlant, PiecewisePolynomial, SceneGraph,
    Parser, JointActuatorIndex, MathematicalProgram, Solve,OsqpSolver
)
from Define_valuable import *


def QP_step(delta_gamma_i,delta_lambda_i,w_gamma_i,w_lambda_i,x0):
    solver = OsqpSolver()
    #x_des = np.array([1,1,1,1])

    prog = MathematicalProgram()
    x = np.zeros((N, num_x), dtype="object")
    u = np.zeros((N, num_u), dtype="object")
    lambda_ = np.zeros((N, num_lambda), dtype="object")
    gamma = np.zeros((N, num_gamma), dtype="object")

    for i in range(N):
        x[i] = prog.NewContinuousVariables(num_x, "x_" + str(i))
        u[i] = prog.NewContinuousVariables(num_u, "u_" + str(i))
        lambda_[i] = prog.NewContinuousVariables(num_lambda, "lambda_" + str(i))
        gamma[i] = prog.NewContinuousVariables(num_gamma, "gamma_" + str(i))

    prog.AddBoundingBoxConstraint(x0, x0, x[0])  # add inital constraint

    for i in range(N):
        if i != N-1:
            expr = x[i+1] - A @ x[i] - B @ u[i] - D @ lambda_[i] - d
            value = np.zeros(expr.shape)
            prog.AddLinearEqualityConstraint(expr,value)

            cost_x = (x[i]).T @ Q @ (x[i])
            cost_u = u[i].T @ R @ u[i]
            prog.AddQuadraticCost(cost_x)
            prog.AddQuadraticCost(cost_u)

        if i == N-1:
            cost_x = (x[i]).T @ QN @ (x[i])
            prog.AddQuadraticCost(cost_x)

        gamma[i] = E @ x[i] + F @ lambda_[i] + H @ u[i] + c

        error_gamma = gamma[i] - delta_gamma_i[i] + w_gamma_i[i]
        cost_gamma = rho * error_gamma.T @ G @error_gamma
        error_lambda = lambda_[i] - delta_lambda_i[i] + w_lambda_i[i]
        cost_lambda =   rho * error_lambda.T @ G @ error_lambda
        prog.AddQuadraticCost(cost_gamma)
        prog.AddQuadraticCost(cost_lambda)

    result = solver.Solve(prog)

    x_mpc = result.GetSolution(x)
    #print(x_mpc)
    z_gamma_mpc = result.GetSolution(gamma)
    z_lambda_mpc = result.GetSolution(lambda_)
    u_mpc = result.GetSolution(u)
    z_gamma_mpc_float = np.vectorize(lambda expr: expr.Evaluate())(z_gamma_mpc)

    #print(f"QP step gamma: {z_gamma_mpc}")
    #print(f"QP step lambda: {z_lambda_mpc}")
    return z_gamma_mpc_float,z_lambda_mpc,x_mpc,u_mpc
