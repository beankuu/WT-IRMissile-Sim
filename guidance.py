import numpy as np

#customs
import data
import mObjects as mobj
from mObjects import Vec3D as vec3
import IRguidance

#!===============================================================
# IR Guidance CALCULATIONS
#!===============================================================

def PID(diff, m):
    """
    PID (proportional-integral-derivative) calculation:
        acceleration = N * Lambda * Velocity
    
    https://en.wikipedia.org/wiki/PID_controller

    Args:
        diff          (mObjects.Vec3D) :          N value of PN equation
        m             (mObjects.MissileObject) :  reference of missile object, carrying variables and constants for calculating PID

    Returns:
        mObjects.Vec3D : PID calculated refined/corrected seeker vector
    """

    e1,e2,e3 = diff, m.PIDe1, m.PIDe2
    P = m.data['g_ga_accelControlProp']*(e1-e2) 
    I = m.intgK*e1 
    D = m.data['g_ga_accelControlDiff']*((e1-e2)-(e2-e3))

    m.PIDe1 = e1        #! modifies m
    m.PIDe2 = e2        #! modifies m
    PID = P+I+D

    #! modifies m
    if vec3.norm(PID) < vec3.norm(P+m.data['g_ga_accelControlIntgLim']*e1+D) and m.intgK <= m.data['g_ga_accelControlIntgLim']:
        m.intgK += m.data['g_ga_accelControlIntg']
    elif vec3.norm(PID) > vec3.norm(P+D) and m.intgK >= 0:
        m.intgK -= m.data['g_ga_accelControlIntg']
    
    return PID


def PN(pNav,losVec,mSpeed):
    """
    PN (Proportional navigation) calculation:
        acceleration = N * Lambda * Velocity
    
    relative speed of target from missile, **but use missile velocity instead**

    https://en.wikipedia.org/wiki/Proportional_navigation

    Args:
        pNav          (float) :          N value of PN equation
        losVec        (mObjects.Vec3D) : Line Of Sight change rage (diff of seeker)
        mSpeed        (float) :          relative speed of missile-target, but use missile velocity instead

    Returns:
        mObjects.Vec3D : requested acceleration vector for missile to move towards target
    """
    return pNav*losVec*mSpeed

#!===============================================================
# ENTRY POINT?
#!===============================================================
def getGuideAccel(m,frame,allObjects):
    """
    entry point for calculating "Auto guidance request Acceleration" of given missile
    steps:
    1. If request exceeds trackrate, skip track
    2. get new seeker vector (seeker_simulator)
    3. get corrected seeker vector (PID)
    4. get request acceleration towards target (PN)
    5. If request exceeds G force limit, limits acceleration

    Args:
        m             (mObjects.MissileObject)   : reference of missile object
        frame         (float)                    : given current frame(time)
        allObjects    ([mObjects.SimObject...])  : Targets in field (TargetObj, Flareobj)

    Returns:
        mObjects.Vec3D : requested acceleration vector for missile to move towards target
    """

    # trackrate Detection
    #if frame*data.dt >= m.data['g_ga_timeOut']:
    if not m.isTrackable:
        rate = 1//(m.data['g_rateMax']*data.dt)
        if rate == 0 or np.mod(frame, rate) == 0: 
            m.isTrackable = True
            return vec3()
    # get Target
    newSVec, m.isLocked = IRguidance.ir_seeker_simulator(m,allObjects)
    # get Acceleration Request
    #PIDResult = PID(newSVec-m.fVec,m) #? newSVec - m.sVec? 
    PIDResult = PID(newSVec-m.sVec,m) #? 
    aCal = PN(m.data['g_ga_propNavMult'],
              vec3.normalize(PIDResult),
              vec3.norm(m.vVec))
    newAccel = aCal
    m.sVec = newSVec
    
    #MaxG Detection
    guideReqAccel = newAccel
    m.isTrackable = False

    return guideReqAccel