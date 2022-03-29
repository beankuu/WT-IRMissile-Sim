import numpy as np

# customs
import guidance as guide # for missile guidance

import data
import mObjects as mobj
from mObjects import Vec3D as vec3

#===============================================
def genTargetMovement1(t,frame):
    gravity = data.getGravity(t.pVec.z)

    ROTATION_RADIUS = 30
    accel_wanted = t.vVec.normalize()
    accel_wanted = ROTATION_RADIUS*accel_wanted.rotate(np.radians(60),'z')
    accel_wanted = accel_wanted.rotate(np.radians(-20),'x')
    #accel_wanted *= ROTATION_RADIUS
    #t.upVec = t.upVec.rotate(angle_inc_per_frame,'z')
    
    if accel_wanted.norm() > 7.5*gravity:
        t.aVec = 7.5*gravity*accel_wanted.normalize()
    else:
        t.aVec = accel_wanted
    
    accel_Thrust = vec3() #Thrust # !temporary
    accel_Drag = -t.vVec.normalize() #Drag # !temporary
    accel_Lift = vec3() #0
    accel_Gravity = vec3() #0
    accel_balance = (accel_Thrust+accel_Drag + accel_Lift+accel_Gravity)

    t.aVec += accel_balance
    t.pVec += t.vVec*data.dt + 0.5*t.aVec*data.dt*data.dt
    t.vVec += t.aVec*data.dt #dt == 1 frame
    if t.vVec.norm() > t.data['maxspeed']:
        t.vVec = t.data['maxspeed']*t.vVec.normalize()
    return t
#*----------------------------------------------------------------------
def genTargetTrajectory(target,f):
    """
    IN: TargetObject, Framenumber, list of all objects(for seeker_simulator)
    OUT: TargetObject at given frame
    """
    return genTargetMovement1(target,f).clone()
#===============================================================
def genFlareTrajectory(flare,f,flareStartTime,target):
    """
    IN: FlareObject, Framenumber(int), flareStartTime(float), targetObject
    OUT: FlareObject at given frame
    """
    startFrame = flareStartTime/data.dt
    endFrame = (flareStartTime+flare.data['flareLiveTime'])/data.dt
    if flare.isFired == False and startFrame <= f:
        flare.pVec = target.pVec
        flare.isFired = True
    elif endFrame >= f:
        flare.isOff = True
    else:
        flare.pVec = vec3(data.INFINITE,data.INFINITE,data.INFINITE)
    return flare.clone()
#===============================================================
def genMissileTrajectory(missile,f,allObjects):
    """
    IN: MissileObject, Framenumber, all objects at current frame=[target,flares]
    OUT: MissileObject at given frame
    """
    return guide.getMissileData(missile,f,allObjects).clone()
#===============================================================
def genPaths(target,missile,flaredataflares):
    """
    IN: targetObject, missileObject, [flare(type)Data, flareTimeList]
    OUT: targetData, missileData, flareData
    """
    flares, flareTimes = flaredataflares

    targetData = []
    missileData = []
    flaresData = [ [] for i in range(len(flareTimes)) ]
    ## per frame
    f = 0
    while f < data.MaxFrame:
        ## calculate
        target = genTargetTrajectory(target,f)
        for i in range(len(flareTimes)):
            flares[i] = genFlareTrajectory(flares[i],f,flareTimes[i],target)
        missile = genMissileTrajectory(missile,f,[target,flares])
        ## append
        targetData.append(target)
        for i in range(len(flareTimes)):
            flaresData[i].append(flares[i])
        missileData.append(missile)

        f += 1
    return targetData, missileData, flaresData
