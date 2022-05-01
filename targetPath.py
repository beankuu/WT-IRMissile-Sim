# from pathGen...
from functools import reduce
import numpy as np
import data
import mObjects as mobj
from mObjects import Vec3D as vec3

# TODO : Temporary workout. Fix after missile guidance workout
def getLiftCoeff(t,AOA):
    """
    get Lift Coefficient of missile at given angle request

    Args:
        missile (mObjects.MissileObject) : missile object
        guideReqAccel (mObjects.Vec3D) : requested Acceleration Vector

    Returns:
        float: Lift Coefficient
    """
    ## Thin airfoil theory? broken at high angle
    #CL = AOA*2*np.pi
    ## near-value of NACA-0015 airfoil wind tunnel graph?
    CL = np.sin(2*AOA)
    return CL

# TODO : Temporary workout. Fix after missile guidance workout
def getDragCoeff(t,CL):
    """
    get Drag Coefficient of missile at given angle request

    Args:
        missile (mObjects.MissileObject) : missile object
        guideReqAccel (mObjects.Vec3D) : requested Acceleration Vector

    Returns:
        float: Drag Coefficient
    """
    #dragCx = t.data['dragCx']
    #K = t.data['CxK'] #Drag lift-induced drag
    CdK = 0.3*CL*CL# !temporary
    return 1+CdK

# TODO : Temporary workout. Fix after missile guidance workout
def genTargetAcceleration(t,frame,accelRequest):
    """
    Generate movement of target, for given acceleration request

    Args:
        t (mObjects.TargetObject) : target object
        frame (float) : time (frame)
        accel_wanted (mObjects.Vec3D) : requested Acceleration Vector

    Returns:
        mObjects.TargetObject : target object at given frame
    """
    gravity = data.getGravity(t.pVec.z)
    if vec3.norm(accelRequest) > 7.5*gravity:
        t.aVec = 7.5*gravity*vec3.normalize(accelRequest)
    else:
        t.aVec = accelRequest
    
    airDensity = data.getAirDensity(t.pVec.z)
    velocity = vec3.norm(t.vVec)
    areaWing = t.data['wingAreaSum']# !temporary
    areaNose = 2*2*np.pi
    wingSpoilerArea = 1
    drag_lift_constant = airDensity * velocity * velocity * 0.5
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
    
    #-------------
    fVec = t.fVec
    rVec = vec3(0,1,0) if fVec == vec3(0,0,-1) else vec3.cross(fVec,vec3(0,0,-1))
    upVec = vec3.cross(fVec,rVec)

    accelRequest += fVec-vec3.normalize(t.vVec)
    t.fVec = vec3.normalize(fVec + accelRequest*0.5*data.dt*data.dt)


    fvAngle = np.arccos(vec3.dot(fVec,vec3.normalize(t.vVec)))

    liftCoef = getLiftCoeff(t,fvAngle)
    dragCoef = getDragCoeff(t,liftCoef)

    dragArea = areaNose
    liftArea = areaNose

    liftArea += (areaWing)*np.sin(fvAngle)
    dragArea += (areaWing)*np.sin(fvAngle)

    
    mass = t.data['mass']

    forces = [
        fVec*thrust,
        liftCoef*drag_lift_constant*liftArea*upVec,
        dragCoef*drag_lift_constant*dragArea*(-fVec),
        vec3(0,0,-1)*gravity*mass,
        dragCoef*drag_lift_constant*wingSpoilerArea*vec3.normalize(accelRequest)
    ]
    #print(fvAngle,forces)

    sumAccel = reduce((lambda x,y:x+y), forces)/mass

    return sumAccel

#!===============================================
# Entry Point?
#!===============================================
# TODO : Temporary workout. Fix after missile guidance workout
def genTargetMovement1(t,frame):
    """
    define movement of target

    Args:
        t (mObjects.TargetObject) : target object
        frame (float) : time (frame)

    Returns:
        mObjects.TargetObject : target object at given frame
    """
    accel_wanted = vec3.normalize(t.fVec)
    gravity = data.getGravity(t.pVec.z)

    timenow = frame*data.dt
    if timenow < 0.5:
        accel_wanted = 2*gravity*vec3.normalize(vec3(0,0,-0.1))
    elif timenow < 3.0:
        accel_wanted = 5*gravity*vec3.normalize(vec3(0,1,-0.01))
    elif timenow < 5.0:
        accel_wanted = 2*gravity*vec3.normalize(vec3(0,2,-1))
    elif timenow < 7.0:
        accel_wanted = 5*gravity*vec3.normalize(vec3(1,-1,-10))
    elif timenow < 9.0:
        accel_wanted = 5*gravity*vec3.normalize(vec3(0,-1,-5))
    else:
        accel_wanted = 5*gravity*vec3.normalize(vec3(1,0,-1))

    return genTargetAcceleration(t,frame,accel_wanted)