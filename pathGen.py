from locale import normalize
from re import L
import numpy as np

# customs
import guidance as guide # for missile guidance
import propulsion # for missile acceleration
import targetPath # for target path generation

import data
import mObjects as mobj
from mObjects import Vec3D as vec3

def isHit(t, m):
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
    IN: TargetObject, Framenumber, list of all objects(for seeker_simulator)
    OUT: TargetObject at given frame
    """
    return targetPath.genTargetMovement1(target,f).clone()
#===============================================================
def genFlareTrajectory(flare,f,flareStartTime,target):
    """
    IN: FlareObject, Framenumber(int), flareStartTime(float), targetObject
    OUT: FlareObject at given frame
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
    return flare.clone()
#===============================================================
def genMissileTrajectory(missile,f,allObjects):
    """
    IN: MissileObject, Framenumber, all objects at current frame=[target,flares]
    OUT: MissileObject at given frame
    """
    guideReqAccel = guide.getMissileData(missile,f,allObjects)
    acceleration = propulsion.getAccel(missile,f*data.dt,guideReqAccel)
    #3. Add other forces
    missile.aVec = acceleration
    #4. Get new velocity and position
    missile.pVec += missile.vVec*data.dt + 0.5*missile.aVec*data.dt*data.dt
    missile.vVec += missile.aVec*data.dt
    return missile.clone()
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
