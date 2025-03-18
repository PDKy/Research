import numpy as np


num_x = 4
num_u = 1
num_lambda = 2
num_gamma = num_lambda
n=4
m=2
k=1

N = 10
g = 9.81
mp = 0.411
mc = 0.978
len_p = 0.6
len_com = 0.4267
d1 = 0.35
d2 = -0.35
ks= 100
Ts = 0.01
A = [[0, 0, 1, 0], [0, 0, 0, 1], [0, g*mp/mc, 0, 0], [0, g*(mc+mp)/(len_com*mc), 0, 0]]
A = np.asarray(A)
B = [[0],[0],[1/mc],[1/(len_com*mc)]]
B = np.asarray(B)
D = [[0,0], [0,0], [(-1/mc) + (len_p/(mc*len_com)), (1/mc) - (len_p/(mc*len_com)) ], [(-1 / (mc*len_com) ) + (len_p*(mc+mp)) / (mc*mp*len_com*len_com)  , -((-1 / (mc*len_com) ) + (len_p*(mc+mp)) / (mc*mp*len_com*len_com))    ]]
D = np.asarray(D)
E = [[-1, len_p, 0, 0], [1, -len_p, 0, 0 ]]
E = np.asarray(E)
F = 1/ks * np.eye(2)
F = np.asarray(F)
#c = [[d1], [-d2]]
c= np.array([d1,-d2])

c = np.asarray(c)
d = np.zeros((4))
H = np.zeros((2,1))
A = np.eye(n) + Ts * A
B = Ts*B
D = Ts*D
d = Ts*d

Q = np.array([[10, 0, 0, 0], [0, 3, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
QN = np.array([[10, 0, 0, 0], [0, 3, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

R = np.array([[1]])

qp_weight_gamma,qp_weight_lambda = 100,100
pj_weight_gamma,pj_weight_lambda = 1,1

rho = 1

G=np.eye(num_lambda)

