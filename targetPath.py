# from pathGen...
import numpy as np
import data
import mObjects as mobj
from mObjects import Vec3D as vec3

def getLiftCoeff(t,reqAccel):
    """
    get Lift Coefficient of target at given angle request
    """
    angleReq = np.arccos(
        vec3.dot(vec3.normalize(reqAccel),t.upVec)
    )/(2*np.pi)
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
    CdK = 10*LiftCoeff*LiftCoeff# !temporary
    return 0.1+CdK

def genTargetMovement(t,frame,accel_wanted):
    gravity = data.getGravity(t.pVec.z)
    if vec3.norm(accel_wanted) > 7.5*gravity:
        t.aVec = 7.5*gravity*vec3.normalize(accel_wanted)
    else:
        t.aVec = accel_wanted
    
    airDensity = data.getAirDensity(t.pVec.z)
    velocity = vec3.norm(t.vVec)
    wingArea = t.data['wingAreaSum']# !temporary
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
        vec3(0,0,-1)*gravity*t.data['mass']
    ]

    accel_balance = sum(forceList)/t.data['mass']

    t.aVec += accel_balance
    t.pVec += t.vVec*data.dt + 0.5*t.aVec*data.dt*data.dt
    t.vVec += t.aVec*data.dt #dt == 1 frame
    if vec3.norm(t.vVec) > t.data['maxspeed']:
        t.vVec = t.data['maxspeed']*vec3.normalize(t.vVec)
    t.fVec = vec3.normalize(t.vVec)
    return t

#===============================================

def genTargetMovement1(t,frame):
    """
    IN: TargetObject, Framenumber
    OUT: TargetObject at given frame
    """
    ACCEL_MULTIPLIER = 10
    accel_wanted = vec3.normalize(t.fVec)

    timenow = frame*data.dt
    if timenow < 0.5:
        accel_wanted = 7*ACCEL_MULTIPLIER*accel_wanted
        accel_wanted = vec3.rotate(accel_wanted,np.radians(-20),'z')
        accel_wanted = vec3.rotate(accel_wanted,np.radians(5),'x')
        t.upVec = vec3.rotate(t.upVec,np.radians(5),'x')
    elif timenow < 3.0:
        accel_wanted = 3*ACCEL_MULTIPLIER*accel_wanted
        accel_wanted = vec3.rotate(accel_wanted,np.radians(25),'z')
        accel_wanted = vec3.rotate(accel_wanted,np.radians(-4),'x')
        t.upVec = vec3.rotate(t.upVec,np.radians(-4),'x')
    elif timenow < 5.0:
        accel_wanted = 6*ACCEL_MULTIPLIER*accel_wanted
        accel_wanted = vec3.rotate(accel_wanted,np.radians(80),'z')
        accel_wanted = vec3.rotate(accel_wanted,np.radians(120),'x')
        t.upVec = vec3.rotate(t.upVec,np.radians(120),'x')
    elif timenow < 7.0:
        accel_wanted = 7*ACCEL_MULTIPLIER*accel_wanted
        accel_wanted = vec3.rotate(accel_wanted,np.radians(-40),'z')
        accel_wanted = vec3.rotate(accel_wanted,np.radians(10),'x')
        t.upVec = vec3.rotate(t.upVec,np.radians(10),'x')
    elif timenow < 9.0:
        accel_wanted = 3*ACCEL_MULTIPLIER*accel_wanted
        accel_wanted = vec3.rotate(accel_wanted,np.radians(-20),'z')
        accel_wanted = vec3.rotate(accel_wanted,np.radians(20),'x')
        t.upVec = vec3.rotate(t.upVec,np.radians(20),'x')
    else:
        #accel_wanted = ROTATION_RADIUS*accel_wanted.rotate(np.radians(10),'z')
        #t.upVec = t.upVec.rotate(np.radians(10),'z')
        accel_wanted = vec3.rotate(accel_wanted,np.radians(45),'z')

    return genTargetMovement(t,frame,accel_wanted)