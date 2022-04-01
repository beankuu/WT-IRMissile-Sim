# dot, cross, norm
import numpy as np
import copy

# custom
from data import INFINITE as INFINITE #data.INFINITE

class Vec3D:
    # P = list(3) or Vec3D(3) ...
    def __init__(self, x = None, y = None, z = None):
        self.x = 0 if x == None else x
        self.y = 0 if y == None else y
        self.z = 0 if z == None else z
    # +
    def __add__(self, other):
        if type(other) is Vec3D: #typeother?
            return Vec3D(self.x + other.x, self.y + other.y, self.z + other.z)
        elif type(other) is int or type(other) is float or type(other) is np.float64:
            return Vec3D(self.x + other,self.y + other, self.z + other)
        else:
            return Vec3D(self.x + other[0], self.y + other[1], self.z + other[2])
    def __iadd__(self, other):
        return self + other
    def __radd__(self, other):
        return self + other
    def __pos__(self):
        return self
    # -
    def __sub__(self, other):
        if type(other) is Vec3D: #isinstance?
            return Vec3D(self.x - other.x, self.y - other.y, self.z - other.z)
        elif type(other) is int or type(other) is float or type(other) is np.float64:
            return Vec3D(self.x - other, self.y - other, self.z - other)
        else:
            return Vec3D(self.x - other[0], self.y - other[1], self.z - other[2])
    def __isub__(self, other):
        return self - other
    def __rsub__(self, other):
        return -self + other
    def __neg__(self):
        return -1 * self
    # * scalar only
    def __mul__(self, other):
        if type(other) is not int and type(other) is not float and type(other) is not np.float64:
            raise TypeError('Vec3D mul does not support'+str(type(other))+'.\nonly supports scalar(int or float) operations.')
        return Vec3D(self.x * other, self.y * other, self.z * other)
    def __imul__(self, other):
        return self * other
    def __rmul__(self, other):
        return self * other
    # / scalar only, no floordiv!
    def __truediv__(self, other):
        if type(other) is not int and type(other) is not float and type(other) is not np.float64:
            raise TypeError('Vec3D div does not support'+str(type(other))+'.\nonly supports scalar(int or float) operations.')
        # some big number
        divider = INFINITE if other == 0 else other
        return Vec3D(self.x/divider, self.y/divider, self.z/divider)
    def __itruediv__(self,other):
        return self / other
    # ==
    def __eq__(self, other):
        if type(other) is Vec3D:
            return self.x == other.x and self.y == other.y and self.z == other.z
        elif other == None:
            return False
        else:
            return self.x == other[0] and self.y == other[1] and self.z == other[2]
    def __req__(self,other):
        return self == other

    #---------------------------------
    # degree in rads, axis = 'x','y','z'
    def rotate(self, rads, axis):
        rot_cos = np.cos(rads)
        rot_sin = np.sin(rads)
        if axis == 'x':
            tempVec = Vec3D(
                self.x,
                rot_cos*self.y - rot_sin*self.z,
                rot_sin*self.y + rot_cos*self.z
            )
        elif axis == 'y':
            tempVec = Vec3D(
                rot_cos*self.x + rot_sin*self.z,
                self.y,
                -rot_sin*self.x + rot_cos*self.z
            )
        else:
            tempVec = Vec3D(
                rot_cos*self.x - rot_sin*self.y,
                rot_sin*self.x + rot_cos*self.y,
                self.z
            )
        return tempVec
    def rotateDegree(self, degree, axis):
        return self.rotate(np.rads(degree),axis)
    def translate(self, P):
        return self + P
    def scale(self, P):
        return self * P
    def dot(self, other):
        result = np.dot([self.x,self.y,self.z],[other.x,other.y,other.z])
        #return 1 if np.abs(result) > 1 else result
        return result
    def cross(self, other):
        result = np.cross([self.x,self.y,self.z],[other.x,other.y,other.z])
        return Vec3D(result[0],result[1],result[2])
        #self.x = result[0]; self.y = result[1]; self.z = result[2]
        #return self #chaining
    def norm(A):
        return float(np.linalg.norm(np.array([A.x,A.y,A.z])))
    def norm(self):
        return float(np.linalg.norm(np.array([self.x,self.y,self.z])))
    def normalize(self):
        self /= self.norm()
        return self

    def setSpherical(self,r,theta,rho):
        """
        inclination(theta)[-pi/2~pi/2], azimuth(rho)[0~pi] in rads
        """
        theta = theta + np.pi/2 #inclination(theta)[-pi/2~pi/2],
        self.x = r*np.cos(rho)*np.sin(theta)
        self.y = r*np.sin(rho)*np.sin(theta)
        self.z = r*np.cos(theta)
        return self
    def setSphericalDegree(self,r,theta,rho):
        return self.setSpherical(r,np.radians(theta),np.radians(rho))
    def setSphericalDegree(self,lst):
        r, theta, rho = lst
        return self.setSpherical(r,np.radians(theta),np.radians(rho))

    def __str__(self):
        return str([self.x, self.y, self.z])
    def __repr__(self):
        return str([self.x, self.y, self.z])
    
    def clone(self):
        return copy.deepcopy(self)
    def toArray(self):
        return np.array([self.x, self.y, self.z])
    def toList(self):
        return [self.x, self.y, self.z]

# root object
class SimObject:
    # position(pVec), velocity(vVec), acceleration(aVec), frontwards(fVec), upwards(upVec), data dictionary(dataDict)
    def __init__(self, pVec=None, vVec=None, aVec=None, fVec = None, upVec=None, data=None):
        self.pVec = Vec3D() if pVec == None else pVec
        self.vVec = Vec3D(1,0,0) if vVec == None else vVec
        self.aVec = Vec3D() if aVec == None else aVec
        self.fVec = Vec3D(1,0,0) if fVec == None else fVec.normalize()
        self.upVec = Vec3D(0,0,1) if upVec == None else upVec.normalize()
        self.data = {} if data == None else data
    def clone(self):
        return copy.deepcopy(self)

# (1x SimObject), 1x fireFlareAt[float,...], 1x isAfterburnerOnAt[[float,float],...]
class TargetObject(SimObject):
    def __init__(self, pVec=None, vVec=None, aVec=None, fVec = None, upVec=None, data=None, fireFlareAt=None, isAfterburnerOnAt=None):
        super().__init__(pVec,vVec,aVec,fVec,upVec,data)
        self.fireFlareAt = [] if fireFlareAt == None else fireFlareAt
        self.isAfterburnerOnAt = [] if isAfterburnerOnAt == None else isAfterburnerOnAt
        self.isAfterburnerOn = False
        self.isHit = False
    def clone(self):
        return copy.deepcopy(self)

# same as SimObject?
class FlareObject(SimObject):
    def __init__(self, pVec=None, vVec=None, aVec=None, fVec = None, upVec=None, data=None, firedAt=None):
        newpVec = Vec3D(INFINITE,INFINITE,INFINITE) if pVec == None else pVec
        super().__init__(newpVec,vVec,aVec,fVec,upVec,data)
        self.isFired = False
        self.isOff = False
    def clone(self):
        return copy.deepcopy(self)

# (1x SimObject), 1x seekerVec, booleans...
class MissileObject(SimObject):
    # ...seeker direction(sVec), acceleration(aVec)
    def __init__(self, pVec=None, vVec=None, aVec=None, fVec=None, upVec=None, sVec=None, data=None):
        super().__init__(pVec,vVec,aVec,fVec,upVec,data)
        self.sVec = Vec3D() if sVec == None else (sVec-pVec).normalize()
        self.isHit = False
        self.isLocked = True
        self.isMaxG = False
        #self.isMaxSpeed = False #?? (always Thrust > MaxSpeed)
        self.isMaxMach = False
        self.isTrackable = True #for trackrate calculation
        # PID error values
        self.PIDe1=Vec3D()
        self.PIDe2=Vec3D()
        if data == None:
            self.intgK=0.0
        else:
            self.intgK=data['g_ga_accelControlIntg']
    def rotate(self,degree,axis):
        super().rotate(degree,axis)
        self.sVec = self.sVec.rotate(degree,axis)
        return self
    def clone(self):
        return copy.deepcopy(self)