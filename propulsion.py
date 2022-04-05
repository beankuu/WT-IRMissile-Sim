import numpy as np

#customs
import data
import mObjects as mobj
from mObjects import Vec3D as vec3

#!===============================================================
# Force CALCULATIONS
#!===============================================================

#mWingArea = math.pi *math.pow((data.m['caliber'])/2,2)*data.m['wingAreamult']

#mThrust = data.m['force'] 
#mThrustTime =  data.m['timeFire']
#mMass = [data.m['mass'],data.m['massEnd']]
#mDragCx = data.m['dragCx']
#mCxK = data.m['CxK']
#mLength = data.m['length']
#mDistFromCmToStab = data.m['distFromCmToStab'] #(mass center <-> wing) distance
#mCLMax = 2*np.pi*data.m['finsAoA']
"""
Drag(D) = Coeff(Cd) * (air)Density * velocity^2 * (wing)Area * 0.5
Lift(L) = Coeff(CL) * (air)Density * velocity^2 * (wing)Area * 0.5

CDrag = CdK + K(CL - CL0)^2,
where CdK = K*CL^2, (given)CxK = Drag lift-induced-drag (K)

CLMax = 2 * PI * CritAoA(rad)
"""
# Drag = Friction drag + Pressure drag + Compression drag + Lift induced drag
def rotateTorqueForce(m,guideReqAccel):
    """
    function to change upVec and fVec
    
    """
    if guideReqAccel.norm() == 0: return vec3(1,0,0), vec3(0,0,0)
    newFVec = guideReqAccel.normalize()
    if newFVec == m.fVec: return vec3(0,0,0), vec3(0,0,0)
    #newRVec = newFVec.cross(m.fVec)
    #newUpVec = newFVec.cross(newRVec)

    wingDist = m.data['length']*0.5*m.data['distFromCmToStab']
    rvec = wingDist*m.fVec

    #TODO
    cos_theta = m.fVec.dot(newFVec)
    #newForce = (newFVec - m.fVec*cos_theta)/cos_theta
    newForce = (newFVec - m.fVec*cos_theta)*guideReqAccel.norm()
    
    forceTorque = rvec.cross(newForce) #UPVEC?
    
    #m.fVec = newFVec
    #m.upVec = newUpVec
    return newForce, forceTorque
#----------------------------------------
# add thrust towards forward (vVec)
def getThrust(m,time):
    """
    get Thrust at given time
    """
    if time <= m.data['timeFire']:
        return m.data['force']
    else:
        return 0.0

#TODO is it correct...?
def getLift(m,guideReqAccel):
    """
    get Lift Coefficient of missile at given angle request
    """
    angleReq = np.arccos(guideReqAccel.normalize().dot(m.upVec))/(2*np.pi)
    CLMax = m.data['finsAoA']
    return CLMax if angleReq >= CLMax else angleReq

#TODO is it correct...?
def getDrag(m,guideReqAccel):
    """
    get Drag Coefficient of missile at given Lift Coefficient
    """
    # Cd = Cd0 + CL^2 / PI*AR*e
    # AR = s^2 / A
    # s = span, A = wing area, e = efficiency factor
    # elliptical wing e = 1.0, rectangular wing e= 0.7
    # (Total Drag) Cd = (zero lift)Cd0 + (induced drag)Cdi
    angleReq = np.arccos(guideReqAccel.normalize().dot(m.fVec))/(2*np.pi)
    CLMax = m.data['finsAoA'] if angleReq >= m.data['finsAoA'] else angleReq
    
    dragCx = m.data['dragCx'] #(zero lift)Cd0
    K = m.data['CxK'] #K for Cdi(Drag lift-induced drag)
    CdK = K*CLMax*CLMax
    return dragCx+CdK

# return in m/s
def getAccel(m,time,guideReqAccel):
    #-------------------------------------
    # constant calculations
    airDensity = data.getAirDensity(m.pVec.z)
    velocity = m.vVec.norm()
    areaNose = np.pi *np.power((m.data['caliber'])/2,2)
    areaWing = m.data['caliber']*m.data['length']
    totalArea = (areaNose+areaWing)*m.data['wingAreamult']

    drag_constant = airDensity * velocity * velocity * totalArea * 0.5
    lift_constant = airDensity * velocity * velocity * totalArea * 0.5
    #-------------------------------------
    # current time's mass calculation
    if time <= m.data['timeFire']:
        massNow = m.data['mass']-(m.data['mass']-m.data['massEnd'])*(time/m.data['timeFire'])
    else:
        massNow = m.data['massEnd']
    #===================================
    # Change UpVec and FVec!
    #-------------------------------------
    # Climb equation
    # vertical: F sin(c) - D sin(c) + L cos(c) - W = m a
    # horizontal: F cos(c) - D cos(c) + L sin(c) = ma
    #Torque change (fvec change)

    newForce, torque = rotateTorqueForce(m,guideReqAccel)
    #print(result)
    newFVec = 0.5*data.dt*data.dt*newForce/massNow
    newUpVec = 0.5*data.dt*data.dt*torque/massNow
    #print(newForce,torque,newFVec,newUpVec)
    #m.fVec = (newFVec).normalize()
    #m.upVec = (newUpVec).normalize()
    """
    newFVec= (m.fVec + 0.5*torqueAccel*data.dt*data.dt).normalize()
    if newFVec != m.fVec:
        newUpVec = m.fVec.cross(newFVec).normalize()
        m.fVec = newFVec
        m.upVec = vec3(0,0,1) if newUpVec.norm() == 0 else newUpVec
        print(m.fVec,newUpVec)
    #m.upVec = (m.upVec + 0.5*torqueAccel*data.dt*data.dt).normalize()
    """

    #===================================
    #------------
    thrust = getThrust(m,time)
    liftCoef = getLift(m,guideReqAccel)
    dragCoef = getDrag(m,guideReqAccel)
    gravity = data.getGravity(m.pVec.z)
    #===================================
    # Forces
    forces = [
        thrust*m.fVec,
        liftCoef*lift_constant*m.upVec,
        dragCoef*drag_constant*-m.fVec,
        gravity*vec3(0,0,-1)*massNow
    ]
    sumAccel = sum(forces)/massNow

    #------------
    #return m.fVec*Thrust/massNow+autoReqAccel
    #return sum(forceList)/massNow + autoReqAccel
    #return m.fVec*thrust/massNow+guideReqAccel
    return sumAccel