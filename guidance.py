import math
import numpy as np

#customs
import data
import mObjects as obj
from mObjects import Vec3D as vec3

#!===============================================================
# Guidance CALCULATIONS
#!===============================================================

##band 1(engine)
#targetEngineTemp = data.t['Thrust'] * data.thrustKgsToInfraRedBrightness
#targetAFEngineTemp = targetEngineTemp + (data.t['AfterburnerBoost'] * data.t['ThrustMult'] -1)* data.t['Thrust'] * data.afterburnerThrustKgsToInfraRedBrightness

##band 2(airframe)

## JUST TRACK TARGET
#fov = data.m['g_fov']
#angleMax = data.m['g_angleMax']
#maxG = data.m['g_ga_reqAccelMax']
#machMax = data.m['machMax']
#baseIndSpeed = data.m['g_ga_baseIndSpeed']
#trackRate = data.FPS/data.m['g_rateMax'] # /frame
#guideTimeout = data.m['g_ga_timeOut']

#pNav = data.m['g_ga_propNavMult']
#pid_prop = data.m['g_ga_accelControlProp']
#pid_intg = data.m['g_ga_accelControlIntg']
#pid_intgMax = data.m['g_ga_accelControlIntgLim']
#pid_diff = data.m['g_ga_accelControlDiff']
#data.getLinear(lst,nearkey)

"""
* FOV Simulator
- IN(Fixed): fov, angleMax
- IN(Param): (old)missile_position, (old)missile_seeker_vector, (new)target_position
- OUT: (new)seeker_vector, isLocked
"""
def fov_simulator(m,t):
    #fov_angle = np.rad2deg(np.arccos(m.sVec.dot(t.pVec.normalize())))
    target_angle = np.rad2deg(np.arccos(m.sVec.dot(m.vVec.normalize())))
    if np.abs(target_angle) <= m.data['g_angleMax']:
        #isLocked = True
        return (t.pVec - m.pVec).normalize(), True
    else:
        #isLocked = False
        return m.sVec, False
"""
* Proportional Navigation
- a_vec = (-N * rel_speed * missile_direction) cross LOS
- regard "relative speed" as "missile speed"
- IN(Fixed): N
- IN(Param): line_of_sight, missile_speed
- OUT: acceleration_vector
"""
def PN(pNav,locVec,mSpeed):
    return pNav*locVec*mSpeed
"""
* PID
- Goal? : 0 acceleration (correct course)
- In(Fixed): pid_prop, pid_intg, pid_intgMax, pid_diff
- In(Param): 
- Out: refined?_acceleration_vector
"""
def PID(aCal, m):
    GOAL = vec3(0,0,0) #0 acceleration
    e1,e2,e3 = GOAL-aCal, m.PIDe1, m.PIDe2
    P = m.data['g_ga_accelControlProp']*(e1-e2) 
    I = m.intgK*e1 
    D = m.data['g_ga_accelControlDiff']*((e1-e2)-(e2-e3))

    m.PIDe1 = e1
    m.PIDe2 = e2
    PID = P+I+D

    if PID.norm() > (P+m.data['g_ga_accelControlIntgLim']*e1+D).norm() and m.intgK < m.data['g_ga_accelControlIntgLim']:
        m.intgK += m.data['g_ga_accelControlIntg']
    elif PID.norm() < (P+D).norm() and m.intgK > 0:
        m.intgK -= m.data['g_ga_accelControlIntg']
    return PID

#!===============================================================
# Force CALCULATIONS
#!===============================================================

#mWingArea = math.pi *math.pow((data.m['caliber'])/2,2)*data.m['wingAreamult']

#mThrust = data.m['force'] 
#mThrustTime =  data.m['timeFire']
#mMass = [data.m['mass'],data.m['massEnd']]
#mDragCx = data.m['dragCx']
#mCxK = data.m['CxK']
#mLength = data.m['length']
#mDistFromCmToStab = data.m['distFromCmToStab'] #(mass center <-> wing) distance
#mCLMax = 2*np.pi*data.m['finsAoA']
"""
Drag(D) = Coeff(Cd) * (air)Density * velocity^2 * (wing)Area * 0.5
Lift(L) = Coeff(CL) * (air)Density * velocity^2 * (wing)Area * 0.5

CDrag = CdK + K(CL - CL0)^2,
where CdK = K*CL^2, (given)CxK = Drag lift-induced-drag (K)

CLMax = 2 * PI * CritAoA(rad)
"""
# add thrust towards forward (vVec)
def getFrontwardsAccel(m,f,dt):
    if f*dt <= m.data['timeFire']:
        mass = m.data['mass']-(m.data['mass']-m.data['massEnd'])*(f*dt/m.data['timeFire'])
        return (m.data['force']/mass) *m.vVec.normalize()
    else:
        return vec3()
#TODO drag
def getBackwardsAccel(m,f,dt):
    return -2*m.vVec.normalize()
#TODO ...how?
def getUpwardsAccel(m,f,dt):
    CLMax = 2*np.pi*m.data['finsAoA']
    return vec3()
#TODO  ...how?
def getDownwardsAccel(m,f,dt):
    return vec3()

def getAccel(m,f,dt):
    #drag_lift_constant = data.getAirDensity(m.pVec.z) * np.power(m.vVec.norm(),2) * mWingArea * 0.5
    return getFrontwardsAccel(m,f,dt)+getBackwardsAccel(m,f,dt)+getUpwardsAccel(m,f,dt)+getDownwardsAccel(m,f,dt)

#!===============================================================
# ENTRY POINT?
#!===============================================================
"""
* get Missile Location(Position) Vector
- IN(Fixed): maxG, maxSpeed, machMax
"""
def getMissileData(m,t,frame,dt):
    newSVec, m.isLocked = fov_simulator(m,t)
    aCal = PN(m.data['g_ga_propNavMult'],
              (newSVec-m.sVec).normalize(),
              m.vVec.norm()*dt)
    m.sVec = newSVec
    PIDResult = aCal+PID(aCal,m)

    m.aVec = getAccel(m,frame,dt)*dt*dt
    #MaxG Detection
    if frame*dt >= m.data['g_ga_timeOut']:
        if m.isTrackable:
            if PIDResult.norm() > m.data['g_ga_reqAccelMax']*data.getGravity(m.pVec.z):
                m.aVec += m.data['g_ga_reqAccelMax']*PIDResult.normalize()
                m.isMaxG = True
            else:
                m.aVec += PIDResult
                m.isMaxG = False
            m.isTrackable = False
        else:
            rate = 1//(m.data['g_rateMax']*dt)
            if rate == 0 or np.mod(frame, rate) == 0: m.isTrackable = True
    m.vVec += m.aVec #dt == 1frame
    m.pVec += m.vVec #dt == 1frame # + 0.5*m.aVec*dt*dt
    return m