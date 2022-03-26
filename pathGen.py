from tkinter import RIGHT
import numpy as np

# customs
import guidance as guide # for missile guidance

import data
import calculate as calc
import mObjects as obj
from mObjects import Vec3D as vec3

#*----------------------------------------------------------------------
INIT_MISSILE_POSITION=[0,0,3000]   # m
#horizontal coordinate(r,inclination,azimuth), degree
INIT_TARGET_HORIZ_COORDINATE=(2500, 90, -20)
TARGET_INIT_VELOCITY = 280 #(m/s), 1008kmh

REACTION_TIME = 1
MAXROTATION = 0.8
ROTATION_RADIUS_INIT = 40
ROTATION_RADIUS_INCREASE = 300
DOWNWARDS_ANGLE = np.radians(0)

def genTargetTrajectory(target):
    dt = data.dt
    t = target
    t.fire_flare_at = 3
    #horizontal -> cartesian coordinate
    t.pVec.setSphericalDegree(INIT_TARGET_HORIZ_COORDINATE)
    # +initial position
    t.pVec += INIT_MISSILE_POSITION 
    t.pVec -= [0,300,300]
    # fly straight forward! (m/s -> m/frame)
    t.vVec = vec3(TARGET_INIT_VELOCITY, 0, 0)*dt#s.rotate(np.radians(30),'z')
    # 1. fly straight, until reaction time
    f = 0
    while f*dt < REACTION_TIME:
        t.pVec += t.vVec # dt == 1 frame
        yield t.clone()
        f += 1

    # 2. rotate
    t.vVec.rotate(np.radians(40),'z')
    angle = 0
    angle_inc_per_frame = MAXROTATION*2*np.pi/(data.MaxFrame-f*dt)
    radius = ROTATION_RADIUS_INIT
    radius_inc_per_frame = MAXROTATION*ROTATION_RADIUS_INCREASE/(data.MaxFrame-f*dt)
    
    while f < data.MaxFrame:
        gravity = data.getGravity(t.pVec.z)
        
        accel_wanted = vec3(0,radius,0)
        accel_wanted = accel_wanted.rotate(-angle,'x')
        t.upVec = t.upVec.rotate(angle_inc_per_frame,'x')
        
        if accel_wanted.norm()/gravity > 7.5:
            t.aVec = 7.5*accel_wanted.normalize()*dt*dt
        else:
            t.aVec = accel_wanted*dt*dt
        
        accel_front = vec3() #Thrust # !temporary
        accel_back = -t.vVec.normalize() #Drag # !temporary
        accel_up = vec3() #0
        accel_down = vec3() #0
        accel_balance = (accel_front+accel_back + accel_up+accel_down)
        t.aVec += accel_balance*dt*dt
        t.vVec += t.aVec #dt == 1 frame
        t.pVec += t.vVec #dt == 1 frame #+ 0.5*t.aVec
        yield t.clone()
        
        angle += angle_inc_per_frame
        radius += radius_inc_per_frame
        f += 1



#*----------------------------------------------------------------------
def genMissileTrajectory(missile, targetDataList):
    dt = data.dt
    m = missile
    m.pVec += INIT_MISSILE_POSITION
    m.intgK = m.data['g_ga_accelControlIntg']
    m.sVec = (targetDataList[0].pVec-m.pVec).normalize()
    m.vVec = m.sVec.normalize()

    f = 0
    while f < data.MaxFrame:
        m = guide.getMissileData(m,targetDataList[f],f,dt)
        yield m.clone()
        f += 1
    #"""
