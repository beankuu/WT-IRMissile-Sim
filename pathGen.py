from re import L
import numpy as np

# customs
import guidance as guide # for missile guidance
import targetPath # for target path generation

import data
import mObjects as mobj
from mObjects import Vec3D as vec3

def isHit(t, m):
    mtDistance = (m.pVec-t.pVec).norm()
    ## Method 1: minimum distance between two vectors
    vVecCross = m.upVec if m.vVec.normalize() == t.vVec.normalize() else m.vVec.cross(t.vVec)
    maxDx = np.abs(vVecCross.dot(m.pVec-t.pVec)/vVecCross.norm())
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
