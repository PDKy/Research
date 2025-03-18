from Define_valuable import *
import numpy as np

from pydrake.all import (
    DiagramBuilder, Simulator, FindResourceOrThrow, MultibodyPlant, PiecewisePolynomial, SceneGraph,
    Parser, JointActuatorIndex, MathematicalProgram, Solve,OsqpSolver
)

def slackstep(z_gamma_ip1,z_lambda_ip1,delta_gamma_ip1,delta_lambda_ip1,w_gamma_i,w_lambda_i):
    w_gamma_ip1 = np.zeros((N,num_gamma))
    w_lambda_ip1 = np.zeros((N,num_lambda))

    for i in range(N):
        w_gamma_ip1[i] = w_gamma_i[i] + z_gamma_ip1[i] - delta_gamma_ip1[i]
        w_lambda_ip1[i] = w_lambda_i[i] + z_lambda_ip1[i] - delta_lambda_ip1[i]

    return w_gamma_ip1, w_lambda_ip1



