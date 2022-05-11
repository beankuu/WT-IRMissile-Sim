import numpy as np

# customs
import guidance as guide # for missile guidance
import propulsion # for missile acceleration
import targetPath # for target path generation

import data
import mObjects as mobj
from mObjects import Vec3D as vec3

import copy

def isHit(t, m):
    """
    boolean result for : did target and missile hit?

    Args:
        t (mObjects.TargetObject): target object
        m (mObjects.MissileObject): missile object

    Returns:
        boolean : did target and missile hit?
    """
    mtDistance = vec3.norm(m.pVec-t.pVec)
    ## Method 1: minimum distance between two vectors
    vVecCross = vec3(0,0,1) if vec3.normalize(m.vVec) == vec3.normalize(t.vVec) else vec3.cross(m.vVec,t.vVec)
    maxDx1 = np.abs(
        vec3.dot(vVecCross, (m.pVec-t.pVec)/vec3.norm(vVecCross))
    )
    ## Method 2: mindist Point + v*dt as radius
    maxDx2 = (vec3.norm(m.vVec) if vec3.norm(t.vVec) < vec3.norm(m.vVec) else vec3.norm(t.vVec)) * data.dt
    # missile? or other types?
    proximityDist = m.data['proximityFuse_radius']
    #print(maxDx)
    #maxDx = min(maxDx1,maxDx2)
    maxDx = maxDx2
    return mtDistance <= proximityDist+maxDx #?x2?

def isGrounded(obj):
    """
    is object below ground?
    == is z value of SimObject < 0?
    
    Args:
        obj (mObjects.SimObject) : object to find out z value
    
    Returns:
        boolean : true if under-ground else false
    """
    return obj.pVec.z <= 0

def genTargetTrajectory(target,f):
    """
    generating Target Object Data at given frame and target

    Args:
        t (mObjects.TargetObject): target object
        f (frame) : current frame(time)

    Returns:
        mObjects.TargetObject: copy of generated Target Object Data
    """
    # if object hit ground, further calculation is useless
    if isGrounded(target):
        target.aVec = target.pVec = target.vVec = vec3()
        return copy.deepcopy(target)

    # Afterburner?
    timenow = f*data.dt
    closestPair = min(target.isAfterburnerOnAt , key=lambda x: min(abs(y - timenow) for y in x))
    if closestPair:
        if timenow > closestPair[0]:
            target.isAfterburnerOn = True
        if timenow > closestPair[1]:
            target.isAfterburnerOn = False
    
    # temporary workaround
    if True:
        po = target.pVec
        ## left bank
        #target.pVec += vec3(np.cos(f/350)*data.dt*1000/3.6,np.sin(f/150)*data.dt*100/3.6,0) #sim1, sim2
        #target.pVec += vec3(np.cos(f/350)*data.dt*1000/3.6,np.sin(f/150)*data.dt*30/3.6,0) #sim3
        #target.pVec += vec3(np.cos(f/350)*data.dt*1000/3.6,np.sin(f/150)*data.dt*150/3.6,0) #sim4
        ## barrel roll
        #target.pVec += vec3(700/3.6*data.dt,5*np.cos(f/50),5*np.sin(f/50))*0.7
        ## barrel roll 2
        #target.pVec += vec3(600/3.6*data.dt,3*np.cos(f/40),3*np.sin(f/25))
        ## diagonal 
        #target.pVec += vec3(500/3.6*data.dt,600/3.6*data.dt,0)*0.7
        ## barrel roll custom
        target.pVec += vec3(900/3.6*data.dt,2*np.sin(f/25),2*np.cos(f/40))
        
        target.vVec = (target.pVec-po)/(data.dt)
        return copy.deepcopy(target)

    if True:
        accelTarget = vec3(1,0,0)
    else:
        accelTarget = targetPath.genTargetMovement1(target,f)

    target.aVec += accelTarget
    target.pVec += target.vVec*data.dt #+ 0.5*t.aVec*data.dt*data.dt
    target.vVec += target.aVec*data.dt #dt == 1 frame
    if vec3.norm(target.vVec) > target.data['maxspeed']:
        target.vVec = target.data['maxspeed']*vec3.normalize(target.vVec)
    return copy.deepcopy(target)
#===============================================================
def genFlareTrajectory(flare,f,flareStartTime,target):
    """
    generating Flare Object Data at given frame and flare

    Args:
        flare (mObjects.FlareObject): flare object
        f (frame) : current frame(time)
        flareStartTime (float) : start time(launch time) of flare
        target (mObjects.TargetObject) : target(source) of flare

    Returns:
        mObjects.FlareObject: copy of generated Flare Object Data
    """
    # if object hit ground, further calculation is useless
    if isGrounded(flare):
        flare.aVec = flare.pVec = flare.vVec = vec3()
        return copy.deepcopy(flare)
    
    startFrame = flareStartTime/data.dt
    endFrame = (flareStartTime+flare.data['timeLife'])/data.dt
    if flare.isFired == False and startFrame <= f:
        flare.pVec = target.pVec
        flare.isFired = True
    elif endFrame >= f:
        flare.isOff = True
    else:
        flare.pVec = vec3(data.INFINITE,data.INFINITE,data.INFINITE)
    return copy.deepcopy(flare)
#===============================================================
def genMissileTrajectory(missile,f,allObjects):
    """
    generating Missile Object Data at given frame and missile

    Args:
        missile (mObjects.MissileObject) : missile object
        f (frame) : current frame(time)
        allObjects ([mObjects.SimObject...]) : list of object in field (TargetObject, FlareObject)

    Returns:
        mObjects.MissileObject: copy of generated Missile Object Data
    """
    # if object hit ground, further calculation is useless
    if isGrounded(missile):
        missile.aVec = missile.pVec = missile.vVec = vec3()
        return copy.deepcopy(missile)
    if missile.isHit:
        guideReqAccel = vec3()
    else:
        guideReqAccel = guide.getGuideAccel(missile,f,allObjects)
    acceleration = propulsion.getAccel(missile,f*data.dt,guideReqAccel)
    #3. Add other forces
    missile.aVec = acceleration
    #4. Get new velocity and position
    missile.pVec += missile.vVec*data.dt #+ 0.5*missile.aVec*data.dt*data.dt
    missile.vVec += missile.aVec*data.dt
    return copy.deepcopy(missile)
#===============================================================
#!===============================================================
# ENTRY POINT
#!===============================================================
def genPaths(target,missiles,flaredataflares):
    """
    generating path Object data of target,missile,flares

    Args:
        target  (mObjects.TargetObject) : target Object
        missiles [(mObjects.MissileObject)] : missile objects
        flaredataFlares ([mObjects.flareObject...] : list of flare objects

    Returns:
        [mObjects.targetObject...] : list of Target Object Data, per frame
        [[mObjects.MissileObject...]...] : list of (list of Missile Object Data, per frame) per missile obj
        [[mObjects.FlareObject...]...] : list of (list of Flare Object Data, per frame) per Flare Object
    """
    flares, flareTimes = flaredataflares

    targetData = []
    missileData = [ [] for i in range(len(missiles)) ]
    flaresData = [ [] for i in range(len(flareTimes)) ]
    
    wasHit = False
    ## per frame
    f = 0
    while f < data.MaxFrame:
        ## calculate
        target = genTargetTrajectory(target,f)
        for i in range(len(flareTimes)):
            flares[i] = genFlareTrajectory(flares[i],f,flareTimes[i],target)
        for i in range(len(missiles)):
            missiles[i] = genMissileTrajectory(missiles[i],f,[target,flares])
            if isHit(target,missiles[i]):
                target.isHit = True; missiles[i].isHit = True
        ## append
        targetData.append(target)
        for i in range(len(flareTimes)):
            flaresData[i].append(flares[i])
        for i in range(len(missiles)):
            missileData[i].append(missiles[i])

        f += 1
    return targetData, missileData, flaresData
