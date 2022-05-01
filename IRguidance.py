import numpy as np

#customs
import data
import mObjects as mobj
from mObjects import Vec3D as vec3

#!===============================================================
# IR Guidance CALCULATIONS
#!===============================================================

def seeker_simulator(m, allObjects):
    """
    simulates seeker of given missile, calculating which target to track

    Args:
        m             (mObjects.MissileObj):      missile to simulate (source)
        allObjects    ([mObjects.SimObject...]):  Targets in field (TargetObj, Flareobj)

    Returns:
        mObjects.Vec3D : unit vector towards acquired target (seeker Vector)
        Boolean :        did missile acquire any target? T/F
    """
    #fov_angle = np.rad2deg(np.arccos(m.sVec.dot(t.pVec.normalize())))
    target, flares = allObjects
    objects = flares + [target]
    # 1. select target from list, which is in fov range
    targetList = []
    for obj in objects:
        calcRange = vec3.norm(obj.pVec - m.pVec)
        calcAngle = np.abs(np.rad2deg(
            np.arccos(
                vec3.dot(vec3.normalize(obj.pVec - m.pVec), m.sVec)
            )))
        if calcRange <= m.data['rangeBand0']:
            # flare resistance
            if m.isLocked:
                if calcAngle <= m.data['g_fov']/2:
                    targetList.append(obj)
            else:
                if calcAngle <= m.data['g_angleMax']/2:
                    targetList.append(obj)
    
    # 2. from targetlist, ignore brightness below 1
    #"""
    brightness = 1 # minimum
    brightestTargets = [] #obj, brightness
    for obj in targetList:
        dist = vec3.norm(obj.pVec - m.pVec)
        if type(obj) == mobj.FlareObject: 
            brightness = obj.data['flareBrightness']
        elif type(obj) == mobj.TargetObject: 
            if obj.isAfterburnerOn: 
                thrust = obj.data['ThrustMult']*obj.data['Thrust']
                afThrust = thrust*(obj.data['AfterburnerBoost']-1)
            else:
                thrust = obj.data['Thrust']
                afThrust = 0
            brightness = thrust*data.thrustKgsToInfraRedBrightness + afThrust*data.afterburnerThrustKgsToInfraRedBrightness
        # brightness decreases 1/r^2
        brightness*=1/(dist*dist)
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
        #"""
    #targetList = [elm[0] for elm in brightestTargets]
    #targetList = brightestTargets
    #print(targetList)
    # no target? get out
    if not targetList:
        return m.sVec, False
    ## 3. get nearset(2) from center
    angle = data.INFINITE
    target = targetList[0]
    for obj in targetList:
        newAngle = np.rad2deg(
            np.arccos(
                vec3.dot(vec3.normalize(obj.pVec - m.pVec),m.sVec)
        ))
        if newAngle < angle:
            angle = newAngle
            target = obj
    
    # 4. get target angle
    target_angle = 0 if m.fVec == m.sVec else np.rad2deg(np.arccos(vec3.dot(m.fVec,m.sVec)))
    #print(target_angle, m.data['g_angleMax'])
    if np.abs(target_angle) <= m.data['g_angleMax']:
        return vec3.normalize(target.pVec - m.pVec), True
    else:
        return m.sVec, False

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
    newSVec, m.isLocked = seeker_simulator(m,allObjects)
    # get Acceleration Request
    #PIDResult = PID(newSVec-m.fVec,m) #? newSVec - m.sVec? 
    PIDResult = PID(newSVec-m.sVec,m) #? 
    aCal = PN(m.data['g_ga_propNavMult'],
              vec3.normalize(PIDResult),
              vec3.norm(m.vVec))
    newAccel = aCal
    m.sVec = newSVec
    
    #MaxG Detection
    accelMax = m.data['g_ga_reqAccelMax']*data.getGravity(m.pVec.z)
    if vec3.norm(newAccel) > accelMax:
        guideReqAccel = accelMax*vec3.normalize(newAccel)
        m.isMaxG = True
    else:
        guideReqAccel = newAccel
        m.isMaxG = False
    m.isTrackable = False

    return guideReqAccel