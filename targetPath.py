# from pathGen...
import numpy as np
import data
import mObjects as mobj
from mObjects import Vec3D as vec3

def getLiftCoeff(t,reqAccel):
    """
    get Lift Coefficient of target at given angle request
    """
    angleReq = np.arccos(reqAccel.normalize().dot(t.upVec))/(2*np.pi)
    AoAMax = t.data['AoA']
    if angleReq >= AoAMax:
        return AoAMax
    else:
        return angleReq

def getDragCoeff(t,LiftCoeff):
    """
    get Drag Coefficient of target at given Lift Coefficient
    ???
    """
    #dragCx = t.data['dragCx']
    #K = t.data['CxK'] #Drag lift-induced drag
    CdK = 30*LiftCoeff*LiftCoeff# !temporary
    return CdK

def genTargetMovement(t,frame,accel_wanted):
    gravity = data.getGravity(t.pVec.z)
    if accel_wanted.norm() > 7.5*gravity:
        t.aVec = 7.5*gravity*accel_wanted.normalize()
    else:
        t.aVec = accel_wanted
    
    airDensity = data.getAirDensity(t.pVec.z)
    velocity = t.vVec.norm()
    wingArea = t.data['wingAreaSum']/10000# !temporary
    drag_lift_constant = airDensity * velocity * velocity * wingArea * 0.5
    #ThrustMult
    #AfterburnerBoost
    timenow = frame*data.dt
    closestPair = min(t.isAfterburnerOnAt , key=lambda x: min(abs(y - timenow) for y in x))
    if closestPair:
        if timenow > closestPair[0]:
            t.isAfterburnerOn = True
        if timenow > closestPair[1]:
            t.isAfterburnerOn = False

    if t.isAfterburnerOn: 
        thrust = t.data['AfterburnerBoost']*t.data['ThrustMult']* t.data['Thrust']*t.data['enginecount']
    else:
        thrust = t.data['Thrust']*t.data['enginecount'] #Thrust # !temporary
    LiftCoeff = getLiftCoeff(t,accel_wanted)
    DragCoeff = getDragCoeff(t,LiftCoeff)

    forceList = [
        t.fVec*thrust,
        t.upVec*LiftCoeff*drag_lift_constant,
        (-t.fVec)*DragCoeff*drag_lift_constant,
        vec3(0,0,-1)*gravity
    ]

    accel_balance = sum(forceList)/t.data['mass']

    t.aVec += accel_balance
    t.pVec += t.vVec*data.dt + 0.5*t.aVec*data.dt*data.dt
    t.vVec += t.aVec*data.dt #dt == 1 frame
    if t.vVec.norm() > t.data['maxspeed']:
        t.vVec = t.data['maxspeed']*t.vVec.normalize()
    t.fVec = t.vVec.normalize()
    return t

#===============================================

def genTargetMovement1(t,frame):
    """
    IN: TargetObject, Framenumber
    OUT: TargetObject at given frame
    """
    ACCEL_MULTIPLIER = 10
    accel_wanted = t.fVec.normalize()

    timenow = frame*data.dt
    if timenow < 0.5:
        accel_wanted = 7*ACCEL_MULTIPLIER*accel_wanted.rotate(np.radians(-30),'z')
        accel_wanted = accel_wanted.rotate(np.radians(10),'x')
        t.upVec = t.upVec.rotate(np.radians(-10),'x')
    elif timenow < 3.0:
        accel_wanted = 3*ACCEL_MULTIPLIER*accel_wanted.rotate(np.radians(25),'z')
        accel_wanted = accel_wanted.rotate(np.radians(-15),'x')
        t.upVec = t.upVec.rotate(np.radians(-15),'x')
    elif timenow < 5.0:
        accel_wanted = 2*ACCEL_MULTIPLIER*accel_wanted.rotate(np.radians(50),'z')
        accel_wanted = accel_wanted.rotate(np.radians(30),'x')
        t.upVec = t.upVec.rotate(np.radians(30),'x')
    elif timenow < 7.0:
        accel_wanted = 3*ACCEL_MULTIPLIER*accel_wanted.rotate(np.radians(-40),'z')
        accel_wanted = accel_wanted.rotate(np.radians(-5),'x')
        t.upVec = t.upVec.rotate(np.radians(-5),'x')
    elif timenow < 9.0:
        accel_wanted = 3*ACCEL_MULTIPLIER*accel_wanted.rotate(np.radians(-20),'z')
        accel_wanted = accel_wanted.rotate(np.radians(5),'x')
        t.upVec = t.upVec.rotate(np.radians(5),'x')
    else:
        #accel_wanted = ROTATION_RADIUS*accel_wanted.rotate(np.radians(10),'z')
        #t.upVec = t.upVec.rotate(np.radians(10),'z')
        accel_wanted = accel_wanted.rotate(np.radians(45),'z')

    return genTargetMovement(t,frame,accel_wanted)