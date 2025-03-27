from Define_valuable import *
from lemkelcp import lemkelcp

def simulate(x0,u0):

    q = E @ x0 + H @ u0 + c
    M = F
    sol = lemkelcp(F,q,maxIter=700)
    force = np.array(sol[0])

    xip1 = A @ x0 + B @ u0 + D @ force + d

    distance = E @ x0 + F @ force + H @ u0 + c

    print("force",force)
    print("distance",distance)

    return xip1

