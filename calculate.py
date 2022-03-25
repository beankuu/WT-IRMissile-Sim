import numpy as np

def vec_normalize(P1):
    if np.linalg.norm(P1) == 0:
        return P1/99999999
    return P1/np.linalg.norm(P1)
# a = dv/dt
def calcAcceleration(P3,P2,P1,dt):
    V3_2 = (P3-P2)/dt
    V2_1 = (P2-P1)/dt
    return np.linalg.norm(V3_2-V2_1)/(dt)
    #return np.linalg.norm(P3-P1)/(dt*dt)
# v = dx/dt
def calcSpeed(P2,P1,dt):
    return np.linalg.norm(P2-P1)/dt