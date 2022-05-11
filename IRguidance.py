import numpy as np

#customs
import data
import mObjects as mobj
from mObjects import Vec3D as vec3

#!===============================================================
# IR Guidance CALCULATIONS
#!===============================================================

def ir_seeker_simulator(m, allObjects):
    """
    simulates ir seeker of given missile, calculating which target to track

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
            if calcAngle <= m.data['g_fov']/2:
                targetList.append(obj)

    # 2. from targetlist
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
    targetList = [elm[0] for elm in brightestTargets]
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