import numpy as np

from pydrake.all import (
    DiagramBuilder, Simulator, FindResourceOrThrow, MultibodyPlant, PiecewisePolynomial, SceneGraph,
    Parser, JointActuatorIndex, MathematicalProgram, Solve
)
from pydrake.autodiffutils import AutoDiffXd
from Define_valuable import *


def LCS(gamma,lambda_):
    constraint_eval = np.zeros((1,), dtype = AutoDiffXd)
    constraint_eval[0] = gamma @ lambda_
    return constraint_eval

def LCShelper(vars):

    gamma = vars[:num_gamma]
    lambda_ = vars[num_gamma:]
    return LCS(gamma, lambda_)

def projection_step(z_gamma_ip1,z_lambda_ip1):
    prog = MathematicalProgram()
    lambda_ = np.zeros((N, num_lambda), dtype="object")
    gamma = np.zeros((N, num_gamma), dtype="object")
    for i in range(N):
        lambda_[i] = prog.NewContinuousVariables(num_lambda, "lambda_" + str(i))
        gamma[i] = prog.NewContinuousVariables(num_gamma, "gamma_" + str(i))

    ub = np.zeros(1)
    lb = np.zeros(1)

    #print("start projection step")
    for i in range(N):
        error_gamma = gamma[i] - z_gamma_ip1[i]
        error_lambda = lambda_[i] - z_lambda_ip1[i]
        cost_gamma = pj_weight_gamma * error_gamma.T @ error_gamma
        cost_lambda = pj_weight_lambda * error_lambda.T @ error_lambda


        prog.AddQuadraticCost(cost_gamma)
        prog.AddQuadraticCost(cost_lambda)
        prog.AddConstraint(LCShelper,lb,ub,np.hstack([gamma[i],lambda_[i]]))

        zero_vec = np.zeros_like(gamma[i])
        inf_vec = np.full_like(gamma[i], np.inf)

        prog.AddBoundingBoxConstraint(zero_vec,inf_vec, gamma[i])
        prog.AddBoundingBoxConstraint(zero_vec, inf_vec, lambda_[i])

    result = Solve(prog)
    delta_gamma_mpc = result.GetSolution(gamma)
    delta_lambda_mpc = result.GetSolution(lambda_)
    #print(f"project step gamma: {delta_gamma_mpc}")
    #print(f"project step lambda: {delta_lambda_mpc}")


    return delta_gamma_mpc, delta_lambda_mpc
