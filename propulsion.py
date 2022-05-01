from functools import reduce
import numpy as np

#customs
import data
import mObjects as mobj
from mObjects import Vec3D as vec3

#!===============================================================
# Force CALCULATIONS
#!===============================================================

def getAngles(fVec,rVec,upVec,targetVec):
    #nX = fVec* vec3.dot(targetVec,fVec)
    #nY = rVec*vec3.dot(targetVec,rVec)
    #nZ = upVec*vec3.dot(targetVec,upVec)

    #angle = np.arccos(vec3.dot(targetVec,vec3.normalize(nX+nY)))

    #angle = np.arccos(vec3.dot(targetVec,vec3.normalize(fVec)))
    angle = np.arctan(vec3.dot(targetVec,upVec))
    #if vec3.dot(targetVec,vec3.normalize(nX+nY))*angle < 0: angle *=-1
    #if vec3.dot(targetVec,vec3.normalize(fVec)) < 0: angle *=-1
    #if vec3.dot(targetVec,upVec) > 0: angle*=-1

    #print(vec3.dot(targetVec,upVec),vec3.dot(targetVec,vec3.normalize(nX+nY)), angle)

    return angle


def limitDragDirection(m, guideReqAccel):
    """
    Args:
        missile (mObjects.MissileObject) : missile object
        fVec (mObjects.Vec3D) : forward Vector of missile
        rVec (mObjects.Vec3D) : rightwards Vector of missile
        upVec (mObjects.Vec3D) : upwards Vector of missile
        fAngleReq (float) : horizontal angle request
        upAngleReq (float) : vertical angle request
        guideReqAccel (mObjects.Vec3D) : raw acceleration request

    Returns:
        mObjects.Vec3D: drag Force vector
    """
    #fVec = vec3.normalize(m.vVec)
    fVec = vec3.normalize(m.fVec)
    upVec = m.upVec
    rVec = vec3.normalize(vec3.cross(upVec,fVec))

    if vec3.norm(guideReqAccel) == 0: return vec3()
    AOSMax = m.data['finsAoA']
    AOAMax = m.data['finsAoA']

    dragReqDir = vec3.normalize(guideReqAccel)

    nX = fVec *vec3.dot(dragReqDir,fVec)
    nY = rVec *vec3.dot(dragReqDir,rVec)
    nZ = upVec *vec3.dot(dragReqDir,upVec)

    AOScos = vec3.dot(dragReqDir,rVec)
    AOAcos = vec3.dot(dragReqDir,upVec)

    AOSMax1 = rVec * np.cos(AOSMax)
    AOSMax2 = -AOSMax1
    AOAMax1 = upVec * np.cos(AOAMax)
    AOAMax2 = -AOAMax1

    if vec3.norm(nY) > np.tan(AOSMax):
        nY = AOSMax1 if AOScos > 0 else nY
        nY = AOSMax2 if AOScos < 0 else nY
    if vec3.norm(nZ) > np.tan(AOAMax):
        nZ = AOAMax1 if AOAcos > 0 else nZ
        nZ = AOAMax2 if AOAcos < 0 else nZ

    ## direction
    newDragVec = vec3.normalize(nX+nY+nZ)

    return newDragVec


#----------------------------------------
# add thrust towards forward (vVec)
def getThrust(m,time):
    """
    get Thrust of missile at given time

    Args:
        missile (mObjects.MissileObject) : missile object
        time (float) : current time in seconds

    Returns:
        float : scalar thrust force
    """
    if time <= m.data['timeFire']:
        return m.data['force']
    else:
        return 0.0

#TODO is it correct...?
def getLiftCoeff(m,AOA):
    """
    get Lift Coefficient of missile at given angle request

    Args:
        missile (mObjects.MissileObject) : missile object
        guideReqAccel (mObjects.Vec3D) : requested Acceleration Vector

    Returns:
        float: Lift Coefficient
    """
    ## Thin airfoil theory? broken at high angle
    #AOSMax = m.data['finsAoA']*2*np.pi
    #AOAMax = m.data['finsAoA']*2*np.pi
    CL = AOA*2*np.pi
    
    #CL = min(CL,AOAMax,AOSMax) if CL > 0 else max(CL,-AOAMax,-AOSMax)

    ## near-value of NACA-0015 airfoil wind tunnel graph?
    #CL = np.sin(2*AOA)

    #if np.abs(AOA) > m.data['finsAoA'] or np.abs(AOA) > m.data['finsAoA']:
    #    CL = 0
    
    return CL

#TODO is it correct...?
# Drag = Friction drag + Pressure drag + Compression drag + Lift induced drag
def getDragCoeff(m,CL):
    """
    get Drag Coefficient of missile at given angle request

    Args:
        missile (mObjects.MissileObject) : missile object
        guideReqAccel (mObjects.Vec3D) : requested Acceleration Vector

    Returns:
        float: Drag Coefficient
    """
    # Cd = Cd0 + CL^2 / PI*AR*e
    # AR = s^2 / A
    # s = span, A = wing area, e = efficiency factor
    # elliptical wing e = 1.0, rectangular wing e= 0.7
    # (Total Drag) Cd = (zero lift)Cd0 + (induced drag)Cdi

    #CL = AOA*2*np.pi
    dragCx = m.data['dragCx'] #(zero lift)Cd0
    K = 1/m.data['CxK'] #K for Cdi(Drag lift-induced drag) #????
    CdK = K*CL*CL
    return dragCx+CdK

#! ============================================================
# Entry point
#! ============================================================
def getAccel(m,time,guideReqAccel):
    """
    calculate acceleration of missile, at given time

    Args:
        m (mObjects.MissileObject) : missile object
        time (float) : time (s)
        guideReqAccel (mObjects.Vec3D) : requested Acceleration Vector

    Returns:
        mObjects.Vec3D: real acceleration of missile, at given time
    """
    #print(round(time,2))
    fVec = m.fVec
    upVec = m.upVec
    rVec = vec3.normalize(vec3.cross(upVec,fVec))
    #rVec = vec3.normalize(vec3(0,1,0) if fVec == vec3(0,0,-1) else vec3.cross(fVec,vec3(0,0,-1)))
    #upVec = vec3.normalize(vec3.cross(fVec,rVec))
    #m.upVec = upVec
    
    #-------------------------------------
    # current time's mass calculation
    if time <= m.data['timeFire']:
        massNow = m.data['mass']-(m.data['mass']-m.data['massEnd'])*(time/m.data['timeFire'])
    else:
        massNow = m.data['massEnd']
    
    #-------------------------------------
    # Drag-Lift calculation
    airDensity = data.getAirDensity(m.pVec.z)
    velocity = vec3.norm(m.vVec)
    areaNose = np.pi *np.power(m.data['caliber']/2,2)
    areaBody = m.data['caliber']*m.data['length']
    areaTotal = (areaBody)*m.data['wingAreamult']
    areaWing = areaTotal-areaBody
    lift_drag_constant = airDensity * velocity * velocity * 0.5 

    torqueRatio = (m.data['distFromCmToStab']/(m.data['length']/2))

    #===================================
    dragDirection = -vec3.normalize(m.vVec)
    dragRVec = vec3.normalize(vec3(0,1,0) if dragDirection == m.upVec else vec3.cross(m.upVec,dragDirection))
    liftDirection = vec3.normalize(vec3.cross(dragDirection,dragRVec))
    
    vfvec = vec3.normalize(m.vVec)
    vrvec = vec3(0,1,0) if vfvec == upVec else vec3.cross(upVec,vfvec)
    vupvec = vec3.cross(vfvec,vrvec)

    wingDragDirection = limitDragDirection(m,guideReqAccel)

    angle = getAngles(vfvec,vrvec,vupvec,fVec)
    wingvAngle = getAngles(vfvec,vrvec,vupvec,wingDragDirection)

    liftArea = areaWing #Wing Area only!
    dragBodyArea = areaNose*np.cos(angle)+areaBody*np.sin(np.abs(angle)) #Reference Area!
    dragWingArea = areaWing*np.sin(np.abs(wingvAngle))

    liftCoef = getLiftCoeff(m,angle)
    dragCoefBody = getDragCoeff(m,getLiftCoeff(m,angle))
    dragCoefWing = getDragCoeff(m,getLiftCoeff(m,wingvAngle))

    #-------------------------------------
    # rotation force calculation
     
    dragWingForce = dragCoefWing*lift_drag_constant*dragWingArea*wingDragDirection#*2

    #-------------------------------------------
    #------------
    thrust = getThrust(m,time)
    gravity = data.getGravity(m.pVec.z)
    liftForce = liftCoef*lift_drag_constant*liftArea*liftDirection
    dragBodyForce = dragCoefBody*lift_drag_constant*dragBodyArea*dragDirection

    #===================================
    # Forces
    forces = [
        thrust*fVec,
        gravity*vec3(0,0,-1)*massNow,
        liftForce,
        dragBodyForce,
        dragWingForce
    ]

    sumAccel = reduce((lambda x,y:x+y), forces)/massNow

    #TODO rotational force to match fvec == svec
    rotateAccel = sumAccel
    fTanAccel = fVec * vec3.dot(rotateAccel,fVec)
    rTanAccel = rVec * vec3.dot(rotateAccel,rVec)
    upTanAccel = upVec * vec3.dot(rotateAccel,upVec)
    tangentForce = (rTanAccel+upTanAccel)
    #sfdirection = vec3.normalize(m.sVec-m.fVec)
    #m.fVec = vec3.normalize(m.fVec   + 0.5*1/torqueRatio*sfdirection*0.5*data.dt*data.dt) #!TODO
    #m.sVec = vec3.normalize(m.sVec   - 0.5*1/torqueRatio*sfdirection*0.5*data.dt*data.dt) #!TODO
    #m.upVec = vec3.normalize(m.upVec + 0.5*1/torqueRatio*sfdirection*0.5*data.dt*data.dt) #!TODO

    m.fVec = vec3.normalize(m.fVec   + torqueRatio*guideReqAccel*0.5*data.dt*data.dt) #!TODO
    m.sVec = vec3.normalize(m.sVec   - torqueRatio*guideReqAccel*0.5*data.dt*data.dt) #!TODO
    m.upVec = vec3.normalize(m.upVec + torqueRatio*guideReqAccel*0.5*data.dt*data.dt) #!TODO

    #m.fVec = vec3.normalize(m.fVec   + torqueRatio*tangentForce*0.5*data.dt*data.dt) #!TODO
    #m.sVec = vec3.normalize(m.sVec   - torqueRatio*tangentForce*0.5*data.dt*data.dt) #!TODO
    #m.upVec = vec3.normalize(m.upVec + torqueRatio*tangentForce*0.5*data.dt*data.dt) #!TODO

    #return sumAccel
    
    ##* temporary measure...
    return thrust*fVec/massNow+guideReqAccel