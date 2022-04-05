# dot, cross, norm
import numpy as np
#import copy

# custom
from data import INFINITE as INFINITE #data.INFINITE

class Vec3D:
    def __init__(self, x = None, y = None, z = None):
        self.x = 0 if x == None else x
        self.y = 0 if y == None else y
        self.z = 0 if z == None else z
    #-------------
    # +
    def __add__(self, other):
        if type(other) is not Vec3D:
            raise TypeError('Vec3D add does not support'+str(type(other))+'.\nonly supports operations between Vec3D Object.')
        return Vec3D(self.x + other.x, self.y + other.y, self.z + other.z)
    def __iadd__(self, other):
        return self + other
    def __radd__(self, other):
        return self + other
    def __pos__(self):
        return self
    # -
    def __sub__(self, other):
        if type(other) is not Vec3D:
            raise TypeError('Vec3D sub does not support'+str(type(other))+'.\nonly supports operations between Vec3D Object.')
        return Vec3D(self.x - other.x, self.y - other.y, self.z - other.z)
    def __isub__(self, other):
        return self - other
    def __rsub__(self, other):
        return -self + other
    def __neg__(self):
        return -1 * self
    # * scalar only
    def __mul__(self, other):
        if type(other) not in {int, float, np.float64}:
            raise TypeError('Vec3D mul does not support'+str(type(other))+'.\nonly supports scalar(int or float) operations.')
        return Vec3D(self.x * other, self.y * other, self.z * other)
    def __imul__(self, other):
        return self * other
    def __rmul__(self, other):
        return self * other
    # / scalar only, no floordiv!
    def __truediv__(self, other):
        if type(other) not in {int, float, np.float64}:
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
    #-------------
    def __str__(self):
        return str([self.x, self.y, self.z])
    def __repr__(self):
        return str([self.x, self.y, self.z])
    #-------------
    #def clone(self):
    #    return copy.deepcopy(self)
    def toArray(self):
        return np.array([self.x, self.y, self.z])
    def toList(self):
        return [self.x, self.y, self.z]
    #---------------------------------
    # degree in rads, axis = 'x','y','z'
    @staticmethod
    def rotate(obj, rads, axis):
        rot_cos = np.cos(rads)
        rot_sin = np.sin(rads)
        if axis == 'x':
            tempVec = Vec3D(
                obj.x,
                rot_cos*obj.y - rot_sin*obj.z,
                rot_sin*obj.y + rot_cos*obj.z
            )
        elif axis == 'y':
            tempVec = Vec3D(
                rot_cos*obj.x + rot_sin*obj.z,
                obj.y,
                -rot_sin*obj.x + rot_cos*obj.z
            )
        else:
            tempVec = Vec3D(
                rot_cos*obj.x - rot_sin*obj.y,
                rot_sin*obj.x + rot_cos*obj.y,
                obj.z
            )
        #return tempVec
        self = tempVec
        return self
    @staticmethod
    def rotateDegree(obj, degree, axis):
        return Vec3D.rotate(obj,np.radians(degree),axis)
    @staticmethod
    def translate(obj, V):
        return obj + V
    @staticmethod
    def scale(obj, a):
        return obj * a
    @staticmethod
    def dot(obj1, obj2):
        result = np.dot(obj1.toArray(),obj2.toArray())
        #return 1 if np.abs(result) > 1 else result
        return result
    @staticmethod
    def cross(obj1, obj2):
        result = np.cross(obj1.toArray(),obj2.toArray())
        return Vec3D(result[0],result[1],result[2])
        #self.x = result[0]; self.y = result[1]; self.z = result[2]
        #return self #chaining
    @staticmethod
    def norm(obj):
        return float(np.linalg.norm(obj.toArray()))
    @staticmethod
    def normalize(obj):
        obj /= Vec3D.norm(obj)
        return obj
    #-------------
    @staticmethod
    def spherical(r,theta,rho):
        """
        inclination(theta)[-pi/2~pi/2], azimuth(rho)[0~pi] in rads
        """
        obj = Vec3D()
        #theta = theta + np.pi/2 #inclination(theta)[-pi/2~pi/2],
        obj.x = r*np.cos(rho)*np.sin(theta)
        obj.y = r*np.sin(rho)*np.sin(theta)
        obj.z = r*np.cos(theta)
        return obj
    @staticmethod
    def sphericalDegree(r,theta,rho):
        return Vec3D.spherical(r,np.radians(theta),np.radians(rho))

# root object
class SimObject:
    # position(pVec), velocity(vVec), acceleration(aVec), frontwards(fVec), upwards(upVec), data dictionary(dataDict)
    def __init__(self, pVec=None, vVec=None, aVec=None, fVec = None, upVec=None, data=None):
        self.pVec = Vec3D() if pVec == None else pVec
        self.vVec = Vec3D(1,0,0) if vVec == None else vVec
        self.aVec = Vec3D() if aVec == None else aVec
        self.fVec = Vec3D(1,0,0) if fVec == None else Vec3D.normalize(fVec)
        self.upVec = Vec3D(0,0,1) if upVec == None else Vec3D.normalize(upVec)
        self.data = {} if data == None else data

        self.aileron = 0 #0, %
        self.elevator = 0 #0, %
        self.rudder = 0 #-7, %
        self.Ny = 0 #1
        self.Vy = 0 #0, m/s
        self.Wx = 0 #0, deg/s
        self.AoA = 0 #3.9, deg
        self.AoS = 0 #72.5, deg
        self.TAS = 0 #0, km/h
        self.IAS = 0 #0, km/h
        self.M = 0 #0
        #self.manifold_pressure1 #?
        self.atm = 0 #0.28
        self.pitch1 = 0  # 24, deg
        self.thrust1 = 0  # 1, kgs
        self.efficiency1 = 0  # 0, %
        
        #----------------
        self.speed = 0 #-0.0281
        self.vario = 0 #0
        self.altitude_hour = 0 #61.626
        self.altitude_min = 0 #61.626
        self.altitude_10k = 0 #61.626
        self.aviahorizon_roll = 0 #-0.056145
        self.aviahorizon_pitch = 0 #-12.78846
        self.bank = 0 # 0.056
        self.turn = 0 # 0
        self.compass = 0 #252.50
        self.manifold_pressure = 0 #0.279
        self.head_temperature = 0 #235.59
        self.trimmer = 0 #0
        self.prop_pitch = 0 #0
        self.throttle = 0 #0
        self.clock_sec = 0 #56

# (1x SimObject), 1x fireFlareAt[float,...], 1x isAfterburnerOnAt[[float,float],...]
class TargetObject(SimObject):
    def __init__(self, pVec=None, vVec=None, aVec=None, fVec = None, upVec=None, data=None, fireFlareAt=None, isAfterburnerOnAt=None):
        super().__init__(pVec,vVec,aVec,fVec,upVec,data)
        self.fireFlareAt = [] if fireFlareAt == None else fireFlareAt
        self.isAfterburnerOnAt = [] if isAfterburnerOnAt == None else isAfterburnerOnAt
        self.isAfterburnerOn = False
        self.isHit = False

# same as SimObject?
class FlareObject(SimObject):
    def __init__(self, pVec=None, vVec=None, aVec=None, fVec = None, upVec=None, data=None, firedAt=None):
        newpVec = Vec3D(INFINITE,INFINITE,INFINITE) if pVec == None else pVec
        super().__init__(newpVec,vVec,aVec,fVec,upVec,data)
        self.isFired = False
        self.isOff = False

# (1x SimObject), 1x seekerVec, booleans...
class MissileObject(SimObject):
    # ...seeker direction(sVec), acceleration(aVec)
    def __init__(self, pVec=None, vVec=None, aVec=None, fVec=None, upVec=None, sVec=None, data=None):
        super().__init__(pVec,vVec,aVec,fVec,upVec,data)
        self.sVec = Vec3D() if sVec == None else Vec3D.normalize(sVec-pVec)
        self.isHit = False
        self.isLocked = True
        self.isMaxG = False
        #self.isMaxSpeed = False #?? (always Thrust > MaxSpeed)
        self.isMaxMach = False
        self.isTrackable = True #for trackrate calculation
        # PID error values
        self.PIDe1=Vec3D()
        self.PIDe2=Vec3D()
        self.intgK = 0.0 if data == None else data['g_ga_accelControlIntg']