
from Define_valuable import *
import numpy as np
from QP_step import QP_step
from projection_step import projection_step
from SlackStep import slackstep
from Simulate import simulate

from pydrake.all import (
    DiagramBuilder, Simulator, FindResourceOrThrow, MultibodyPlant, PiecewisePolynomial, SceneGraph,
    Parser, JointActuatorIndex, MathematicalProgram, Solve,SnoptSolver, IpoptSolver
)

#x:[-0.02491208, -0.35959261, -0.24777138, -2.02661032]

iter_solve = 350


delta_gamma_i = np.zeros((N,num_gamma))
delta_lambda_i = np.zeros((N,num_lambda))
w_gamma_i = np.zeros((N,num_gamma))
w_lambda_i = np.zeros((N,num_lambda))
x0 = np.zeros(num_x)

#x0 = np.array([ 0.0308456 , -0.22883082 , -0.07717506 , -1.22602177])  #right then, the data show dis-convergence

#x0 = np.array([ 0.03343251 ,-0.18570774, -0.05767517, -0.99531685]) #for warm start

x0[0] = 0.01
x0[2] = 0.03

# QP step
for j in range(1000):

    for i in range(iter_solve):
        z_gamma_ip1, z_lambda_ip1,x_mpc,u_mpc = QP_step(delta_gamma_i, delta_lambda_i, w_gamma_i, w_lambda_i,x0)

        delta_gamma_ip1,delta_lambda_ip1 = projection_step(z_gamma_ip1, z_lambda_ip1)

        w_gamma_ip1, w_lambda_ip1 = slackstep(z_gamma_ip1, z_lambda_ip1, delta_gamma_ip1, delta_lambda_ip1, w_gamma_i, w_lambda_i)

        delta_gamma_i = delta_gamma_ip1
        delta_lambda_i = delta_lambda_ip1
        w_gamma_i = w_gamma_ip1
        w_lambda_i = w_lambda_ip1

        # print("z_gamma_ip1: ", z_gamma_ip1)
        # print("z_lambda_ip1: ", z_lambda_ip1)


    # print(x_mpc[0])
    # print(u_mpc[0])
    # print(E@x_mpc[0])
    # print(H @ np.array([u_mpc[0]]))
    # print(c)
    #print(u_mpc[0])

    x0 = simulate(x_mpc[0], np.array([u_mpc[0]]))


    print(f"x:{x0}")
    print(f"z_gamma_ip1,{z_gamma_ip1[0]}")
    print(f"z_lambda_ip1,{z_lambda_ip1[0]}")
    print(f"delta_gamma_ip1,{delta_gamma_i[0]}")
    print(f"delta_lambda_ip1,{delta_lambda_i[0]}")
    print("next state")