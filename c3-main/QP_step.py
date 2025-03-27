from Define_valuable import *
import numpy as np

from pydrake.all import (
    DiagramBuilder, Simulator, FindResourceOrThrow, MultibodyPlant, PiecewisePolynomial, SceneGraph,
    Parser, JointActuatorIndex, MathematicalProgram, Solve,SnoptSolver, IpoptSolver
)
from Define_valuable import *
from pydrake.autodiffutils import AutoDiffXd

def gamma_constraint(vars):
    x = vars[:num_x]
    lambda_ = vars[num_x:num_x+num_lambda]
    u = vars[num_x+num_lambda:]
    expr = E @ x+ F @ lambda_ + H @ u + c

    constraint_eval = np.zeros((num_gamma,), dtype=AutoDiffXd)
    for i in range(num_gamma):
        constraint_eval[i] = expr[i]
    return constraint_eval


xd = np.zeros(num_x)


def QP_step(delta_gamma_i,delta_lambda_i,w_gamma_i,w_lambda_i,x0):
    solver = SnoptSolver()
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

    zero_vec = np.zeros_like(gamma[0])
    inf_vec = np.full_like(gamma[0], np.inf)


    for i in range(N):
        if i != N-1:
            expr = x[i+1] - A @ x[i] - B @ u[i] - D @ lambda_[i] - d
            value = np.zeros(expr.shape)
            prog.AddLinearEqualityConstraint(expr,value)

            cost_x = (x[i]-xd).T @ Q @ (x[i]-xd)
            cost_u = u[i].T @ R @ u[i]
            prog.AddQuadraticCost(cost_x)
            prog.AddQuadraticCost(cost_u)


        if i == N-1:
            cost_x = (x[i]-xd).T @ QN @ (x[i]-xd)
            prog.AddQuadraticCost(cost_x)

        #expr_gamma =  E @ x[i] + F @ lambda_[i] + H @ u[i] + c

        gamma[i] = E @ x[i] + F @ lambda_[i] + H @ u[i] + c

        prog.AddConstraint(gamma_constraint, zero_vec, inf_vec, np.hstack([x[i],lambda_[i],u[i]]))

        prog.AddBoundingBoxConstraint(zero_vec, inf_vec, lambda_[i])

        error_gamma = gamma[i] - delta_gamma_i[i] + w_gamma_i[i]
        cost_gamma = rho * error_gamma.T @ G @error_gamma
        error_lambda = lambda_[i] - delta_lambda_i[i] + w_lambda_i[i]
        cost_lambda =   rho * error_lambda.T @ G @ error_lambda
        prog.AddQuadraticCost(cost_gamma)
        prog.AddQuadraticCost(cost_lambda)

    result = solver.Solve(prog)

    x_mpc = result.GetSolution(x)
    u_mpc = result.GetSolution(u)
    z_lambda_mpc = result.GetSolution(lambda_)
    #print(x_mpc)
    # for i in range(N):
    #     print( F @ z_lambda_mpc[i] + H * u_mpc[i] + c)

    z_gamma_mpc = result.GetSolution(gamma)

    z_gamma_mpc_float = np.vectorize(lambda expr: expr.Evaluate())(z_gamma_mpc)

    #print(f"QP step gamma: {z_gamma_mpc}")
    #print(f"QP step lambda: {z_lambda_mpc}")
    return z_gamma_mpc_float,z_lambda_mpc,x_mpc,u_mpc
