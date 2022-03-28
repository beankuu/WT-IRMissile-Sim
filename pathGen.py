from tkinter import RIGHT
import numpy as np

# customs
import guidance as guide # for missile guidance

import data
import calculate as calc
import mObjects as obj
from mObjects import Vec3D as vec3

#===============================================
# some fixed values
#-----------------------------------------------
INIT_MISSILE_POSITION=[0,0,3000]   # m

#===============================================
def genTargetMovement1(t,frame,dt,duration):
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
    t.vVec += t.aVec*dt #dt == 1 frame
    if t.vVec.norm() > t.data['maxspeed']:
        t.vVec = t.data['maxspeed']*t.vVec.normalize()
    t.pVec += t.vVec*dt #dt == 1 frame #+ 0.5*t.aVec
    return t
#*----------------------------------------------------------------------
def genTargetTrajectory(target,dt,maxFrame):
    #----------------------------------------
    t = target
    t.fire_flare_at = 3 #s

    #horizontal coordinate(r,inclination[-pi/2~pi/2],azimuth), degree
    t.pVec.setSphericalDegree([2500, -10, -10])
    t.pVec += INIT_MISSILE_POSITION
    # fly straight parallel to x axis! 280m/s = 1008kmh
    t.fVec = vec3(1,0,0)
    t.vVec = vec3(280, 0, 0)    #s.rotate(np.radians(30),'z')

    # 1. fly straight parallel to x axis, until reaction time
    REACTION_TIME = 1.0 #s
    f = 0
    while f*dt < REACTION_TIME:
        t.pVec += t.vVec*dt
        yield t.clone()
        f += 1

    # 2. some movement after reaction time
    startFrame = f*dt
    endFrame = maxFrame
    while f < maxFrame:
        t = genTargetMovement1(t,f, dt, [startFrame,endFrame])
        yield t.clone()
        f += 1
#*===============================================================
def genMissileTrajectory(missile, dt, maxFrame, targetDataList):
    m = missile
    m.pVec += INIT_MISSILE_POSITION
    m.intgK = m.data['g_ga_accelControlIntg']

    m.sVec = (targetDataList[0].pVec-m.pVec).normalize()
    m.fVec = m.sVec.normalize() #front towards target

    f = 0
    while f < maxFrame:
        m = guide.getMissileData(m,targetDataList[f],f,dt)
        yield m.clone()
        f += 1
    #"""
