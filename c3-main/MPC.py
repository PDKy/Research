
from Define_valuable import *
import numpy as np
from QP_step import QP_step
from projection_step import projection_step
from SlackStep import slackstep
from Simulate import simulate

from pydrake.all import (
    DiagramBuilder, Simulator, FindResourceOrThrow, MultibodyPlant, PiecewisePolynomial, SceneGraph,
    Parser, JointActuatorIndex, MathematicalProgram, Solve
)

iter_solve = 500

delta_gamma_i = np.zeros((N,num_gamma))
delta_lambda_i = np.zeros((N,num_lambda))
w_gamma_i = np.zeros((N,num_gamma))
w_lambda_i = np.zeros((N,num_lambda))
x0 = np.zeros(num_x)
x0[0] = 0.01
x0[2] = 0.03

# QP step
for j in range(300):

    for i in range(iter_solve):
        z_gamma_ip1, z_lambda_ip1,x_mpc,u_mpc = QP_step(delta_gamma_i, delta_lambda_i, w_gamma_i, w_lambda_i,x0)

        delta_gamma_ip1,delta_lambda_ip1 = projection_step(z_gamma_ip1, z_lambda_ip1)

        w_gamma_ip1, w_lambda_ip1 = slackstep(z_gamma_ip1, z_lambda_ip1, delta_gamma_ip1, delta_lambda_ip1, w_gamma_i, w_lambda_i)

        #print(z_gamma_ip1, z_lambda_ip1,delta_gamma_ip1,delta_lambda_ip1)

        delta_gamma_i = delta_gamma_ip1
        delta_lambda_i = delta_lambda_ip1
        w_gamma_i = w_gamma_ip1
        w_lambda_i = w_lambda_ip1
    # print(x_mpc[0])
    # print(u_mpc[0])
    # print(E@x_mpc[0])
    # print(H @ np.array([u_mpc[0]]))
    # print(c)

    x0 = simulate(x_mpc[0], np.array([u_mpc[0]]))
    print(f"x:{x0}")
    print(f"z_gamma_ip1,{z_gamma_ip1}")
    print(f"z_lambda_ip1,{z_lambda_ip1}")
    print(f"delta_gamma_i,{delta_gamma_i}")
    print(f"delta_lambda_i,{delta_lambda_i}")


