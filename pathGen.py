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
    vVecCross = m.upVec if vec3.normalize(m.vVec) == vec3.normalize(t.vVec) else vec3.cross(m.vVec,t.vVec)
    maxDx = np.abs(
        vec3.dot(vVecCross, (m.pVec-t.pVec)/vec3.norm(vVecCross))
    )
    ## Method 2: mindist Point + v*dt as radius
    #maxDx = (m.vVec.norm() if t.vVec.norm() < m.vVec.norm() else t.vVec.norm()) * data.dt
    # missile? or other types?
    proximityDist = m.data['proximityFuse_radius']
    #print(mtDistance, proximityDist,maxDx)
    return mtDistance <= proximityDist+2*maxDx #?x2?

def genTargetTrajectory(target,f):
    """
    generating Target Object Data at given frame and target

    Args:
        t (mObjects.TargetObject): target object
        f (frame) : current frame(time)

    Returns:
        mObjects.TargetObject: copy of generated Target Object Data
    """
    return copy.deepcopy(targetPath.genTargetMovement1(target,f))
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
    guideReqAccel = guide.getGuideAccel(missile,f,allObjects)
    acceleration = propulsion.getAccel(missile,f*data.dt,guideReqAccel)
    #3. Add other forces
    missile.aVec = acceleration
    #4. Get new velocity and position
    missile.pVec += missile.vVec*data.dt# + 0.5*missile.aVec*data.dt*data.dt
    missile.vVec += missile.aVec*data.dt
    return copy.deepcopy(missile)
#===============================================================
#!===============================================================
# ENTRY POINT
#!===============================================================
def genPaths(target,missile,flaredataflares):
    """
    generating path Object data of target,missile,flares

    Args:
        target  (mObjects.TargetObject) : target Object
        missile (mObjects.MissileObject) : missile object
        flaredataFlares ([mObjects.flareObject...] : list of flare objects

    Returns:
        [mObjects.targetObject...] : list of Target Object Data, per frame
        [mObjects.MissileObject...] : list of Missile Object Data, per frame
        [[mObjects.FlareObject...]...] : list of (list of Flare Object Data, per frame) per Flare Object
    """
    flares, flareTimes = flaredataflares

    targetData = []
    missileData = []
    flaresData = [ [] for i in range(len(flareTimes)) ]
    
    wasHit = False
    ## per frame
    f = 0
    while f < data.MaxFrame:
        ## calculate
        target = genTargetTrajectory(target,f)
        for i in range(len(flareTimes)):
            flares[i] = genFlareTrajectory(flares[i],f,flareTimes[i],target)
        missile = genMissileTrajectory(missile,f,[target,flares])
        if isHit(target,missile):
            wasHit = True
        if wasHit:
            target.isHit = True; missile.isHit = True
        ## append
        targetData.append(target)
        for i in range(len(flareTimes)):
            flaresData[i].append(flares[i])
        missileData.append(missile)

        f += 1
    return targetData, missileData, flaresData
