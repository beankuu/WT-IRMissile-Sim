import numpy as np

#customs
import data
import mObjects as mobj
from mObjects import Vec3D as vec3

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
# Drag = Friction drag + Pressure drag + Compression drag + Lift induced drag

# add thrust towards forward (vVec)
def getThrust(m,time):
    if time <= m.data['timeFire']:
        mass = m.data['mass']-(m.data['mass']-m.data['massEnd'])*(time/m.data['timeFire'])
        return (m.data['force']/mass) *m.fVec.normalize()
    else:
        return vec3()
#TODO drag
def getDrag(m,time,autoReqAccel):
    return -2*m.vVec.normalize()
#TODO ...how?
def getLift(m,time,autoReqAccel):
    CLMax = 2*np.pi*m.data['finsAoA']
    return vec3()
#TODO after getLift
def getGravity(m,time):
    return vec3()

# return in m/s
def getAccel(m,time,autoReqAccel):
    #drag_lift_constant = data.getAirDensity(m.pVec.z) * np.power(m.vVec.norm(),2) * mWingArea * 0.5
    accelList = [
        getThrust(m,time),
        getDrag(m,time,autoReqAccel),
        getLift(m,time,autoReqAccel),
        getGravity(m,time),
        autoReqAccel
    ]
    return sum(accelList)
#!===============================================================
# Guidance CALCULATIONS
#!===============================================================

##band 1(engine)
#targetEngineTemp = data.t['Thrust'] * data.thrustKgsToInfraRedBrightness
#targetAFEngineTemp = targetEngineTemp + (data.t['AfterburnerBoost'] * data.t['ThrustMult'] -1)* data.t['Thrust'] * data.afterburnerThrustKgsToInfraRedBrightness

##band 2(airframe)

def seeker_simulator(m, allObjects):
    """
    * Seeker Simulator
    - IN(Fixed): fov, angleMax, angleMax
    - IN(Param): (old)missile_position, (old)missile_seeker_vector, (new)target_position
    - OUT: (new)seeker_vector, isLocked
    """
    #fov_angle = np.rad2deg(np.arccos(m.sVec.dot(t.pVec.normalize())))
    target, flares = allObjects
    objects = flares + [target]
    # 1. select target from list, which is in fov range
    targetList = []
    for obj in objects:
        calcRange = (obj.pVec - m.pVec).norm()
        calcFov = np.rad2deg(np.arccos((obj.pVec - m.pVec).normalize().dot(m.sVec)))
        if  calcFov <= m.data['g_fov'] and calcRange <= m.data['rangeBand0']:
            targetList.append(obj)
    
    # 2. from targetlist, ignore brightness below 1
    """
    brightness = 1 # minimum
    brightestTargets = [] #obj, brightness
    for obj in targetList:
        dist = (obj.pVec - m.pVec).norm()
        if type(obj) == mobj.FlareObject: 
            brightness = obj.data['flareBrightness']
        elif type(obj) == mobj.TargetObject: 
            thrust = obj.data['Thrust']
            brightness = thrust*data.thrustKgsToInfraRedBrightness
        # brightness decreases 1/r^2
        brightness*=1000/(dist*dist)
        #if brightness < 1: continue
        #else: brightestTargets.append(obj)
        
        #if empty then add
        if not brightestTargets:
            brightestTargets=[[obj, brightness]]; continue
        else:
            for tempObj,tempBrightness in brightestTargets:
                if tempObj != obj:
                    if tempBrightness < brightness:
                        brightestTargets = [[obj, brightness]]; break
                    elif tempBrightness == brightness:
                        brightestTargets.append([obj, brightness])
        """
    #targetList = [elm[0] for elm in brightestTargets]
    #targetList = brightestTargets
    print(targetList)
    # no target? get out
    if not targetList:
        return m.sVec, False
    ## 3. get nearset(2) from center
    angle = data.INFINITE
    target = targetList[0]
    for obj in targetList:
        newAngle = np.rad2deg(np.arccos((obj.pVec - m.pVec).normalize().dot(m.sVec)))
        if newAngle < angle:
            angle = newAngle
            target = obj
    
    # 4. get target angle
    target_angle = np.rad2deg(np.arccos(m.sVec.dot(m.vVec.normalize())))
    if np.abs(target_angle) <= m.data['g_angleMax']:
        return (target.pVec - m.pVec).normalize(), True
    else:
        return m.sVec, False

def PN(pNav,locVec,mSpeed):
    """
    * Proportional Navigation
    - a_vec = (-N * rel_speed * missile_direction) cross LOS
    - regard "relative speed" as "missile speed"
    - IN(Fixed): N
    - IN(Param): line_of_sight, missile_speed
    - OUT: acceleration_vector
    """
    return pNav*locVec*mSpeed

def PID(diff, m):
    """
    * PID
    - Goal? : front vec == velocity vec (diff == 0 )
    - In(Fixed): pid_prop, pid_intg, pid_intgMax, pid_diff
    - In(Param): 
    - Out: refined?_acceleration_vector
    """
    e1,e2,e3 = diff, m.PIDe1, m.PIDe2
    P = m.data['g_ga_accelControlProp']*(e1-e2) 
    I = m.intgK*e1 
    D = m.data['g_ga_accelControlDiff']*((e1-e2)-(e2-e3))

    m.PIDe1 = e1
    m.PIDe2 = e2
    PID = P+I+D

    if PID.norm() < (P+m.data['g_ga_accelControlIntgLim']*e1+D).norm() and m.intgK <= m.data['g_ga_accelControlIntgLim']:
        m.intgK += m.data['g_ga_accelControlIntg']
    elif PID.norm() > (P+D).norm() and m.intgK >= 0:
        m.intgK -= m.data['g_ga_accelControlIntg']
    return PID

#!===============================================================
# ENTRY POINT?
#!===============================================================
"""
* get Missile Location(Position) Vector
- IN(Fixed): maxG, maxSpeed, machMax
"""
def getMissileData(m,frame,allObjects):
    """
    IN: MissileObject, Framenumber, list of all objects(for seeker_simulator)
    OUT: MissileObject at given frame
    """
    #1. GET TARGET
    newSVec, m.isLocked = seeker_simulator(m,allObjects)
    #2. Acceleration Request
    PIDResult = PID(newSVec-m.sVec,m)
    aCal = PN(m.data['g_ga_propNavMult'],
              (PIDResult).normalize(),
              m.vVec.norm())
    newAccel = aCal
    m.sVec = newSVec
    m.fVec = m.sVec
    
    autoReqAccel = vec3()
    #MaxG Detection
    if frame*data.dt >= m.data['g_ga_timeOut']:
        if m.isTrackable:
            accelMax = m.data['g_ga_reqAccelMax']*data.getGravity(m.pVec.z)
            if newAccel.norm() > accelMax:
                autoReqAccel = accelMax*newAccel.normalize()
                m.isMaxG = True
            else:
                autoReqAccel = newAccel
                m.isMaxG = False
            m.isTrackable = False
        else:
            rate = 1//(m.data['g_rateMax']*data.dt)
            if rate == 0 or np.mod(frame, rate) == 0: m.isTrackable = True
    #3. Add other forces
    m.aVec = getAccel(m,frame*data.dt,autoReqAccel)
    #4. Get new velocity and position
    m.pVec += m.vVec *data.dt + 0.5*m.aVec*data.dt*data.dt
    m.vVec += m.aVec *data.dt
    
    return m