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
        #print(self,other)
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
        if type(other) not in {int, float, np.float64, np.int32}:
            raise TypeError('Vec3D mul does not support'+str(type(other))+'.\nonly supports scalar(int or float) operations.')
        return Vec3D(self.x * other, self.y * other, self.z * other)
    def __imul__(self, other):
        return self * other
    def __rmul__(self, other):
        return self * other
    # / scalar only, no floordiv!
    def __truediv__(self, other):
        if type(other) not in {int, float, np.float64, np.int32}:
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
    def toArray(self):
        """
        Vec3D object to numpy.array([x,y,z])

        Args:
            self (mObjects.Vec3D): Vec3D Object

        Returns:
            numpy.array : np.array([x,y,z])
        """
        return np.array([self.x, self.y, self.z])
    def toList(self):
        """
        Vec3D object to List(x,y,z)

        Args:
            self (mObjects.Vec3D): Vec3D Object

        Returns:
            list : [x,y,z]
        """
        return [self.x, self.y, self.z]
    #---------------------------------
    # degree in rads, axis = 'x','y','z'
    @staticmethod
    def rotate(obj, rads, axis):
        """
        rotation of vector, in given angle(rads) and axis
        !temporary method, for generating target's Path
        !remove after shifting targetPath generation to Rodrigues' rotation formula 

        Args:
            obj           (mObjects.Vec3D): Vector to rotate
            rads          (float):  rotate angle in radians
            axis          (string): axis to rotate object, if not given, default is 'z' axis

        Returns:
            mObjects.Vec3D : rotated vector
        """
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
        """
        rotation of vector, in given angle(degree) and axis
        !temporary method, for generating target's Path
        !remove after shifting targetPath generation to Rodrigues' rotation formula 

        Args:
            obj           (mObjects.Vec3D): Vector to rotate
            rads          (float):  rotate angle in degree
            axis          (string): axis to rotate object, if not given, default is 'z' axis

        Returns:
            mObjects.Vec3D : rotated vector
        """
        return Vec3D.rotate(obj,np.radians(degree),axis)
    @staticmethod
    def translate(obj, V):
        """
        translation of vector, for given vector amount
        ! since we're not using matrix, this formula is useless

        Args:
            obj           (mObjects.Vec3D) : Vector to translate
            V             (mObjects.Vec3D) : given vector amount to translate

        Returns:
            mObjects.Vec3D : translated vector
        """
        return obj + V
    @staticmethod
    def scale(obj, a):
        """
        scaling of vector, for given amount
        ! same as scalar multiplication of vector(__mul__)
        ! since we're not using matrix, this formula is useless

        Args:
            obj           (mObjects.Vec3D) : Vector to scale
            a             (float) :          given amount to scale

        Returns:
            mObjects.Vec3D : scaled vector
        """
        return obj * a
    @staticmethod
    def dot(obj1, obj2):
        """
        dot product of 2 Vec3D objects

        Args:
            obj1          (mObjects.Vec3D) : 1st vector for calculating dot product
            obj2          (mObjects.Vec3D) : 2nd vector for calculating dot product

        Returns:
            float : dot product of given 2 vectors
        """
        result = np.dot(obj1.toArray(),obj2.toArray())
        return result
    @staticmethod
    def cross(obj1, obj2):
        """
        cross product of 2 Vec3D objects

        Args:
            obj1          (mObjects.Vec3D) : 1st vector for calculating cross product
            obj2          (mObjects.Vec3D) : 2nd vector for calculating cross product

        Returns:
            mObjects.Vec3D : cross product of given 2 vectors
        """
        result = np.cross(obj1.toArray(),obj2.toArray())
        return Vec3D(result[0],result[1],result[2])
    @staticmethod
    def norm(obj):
        """
        norm of given Vec3D object

        Args:
            obj (mObjects.Vec3D) : given vector

        Returns:
            float : norm of vector object
        """
        return float(np.linalg.norm(obj.toArray()))
    @staticmethod
    def normalize(obj):
        """
        normalized vector for given vector

        Args:
            obj          (mObjects.Vec3D) : given vector

        Returns:
            mObjects.Vec3D : normalized given vector
        """
        obj /= Vec3D.norm(obj)
        return obj
    #-------------
    @staticmethod
    def spherical(r,theta,rho):
        """
        spherical coordinates to cartesian coordinates

        Args:
            r (float) : radius
            theta (theta) : inclination in radians
            rho (float) : azimuth in radians

        Returns:
            mObjects.Vec3D : cartesian coordinate of vector
        """
        obj = Vec3D()
        #theta = theta + np.pi/2 #inclination(theta)[-pi/2~pi/2],
        obj.x = r*np.cos(rho)*np.sin(theta)
        obj.y = r*np.sin(rho)*np.sin(theta)
        obj.z = r*np.cos(theta)
        return obj
    @staticmethod
    def sphericalDegree(r,theta,rho):
        """
        spherical coordinates to cartesian coordinates

        Args:
            r (float) : radius
            theta (theta) : inclination in degrees
            rho (float) : azimuth in degrees

        Returns:
            mObjects.Vec3D : cartesian coordinate of vector
        """
        return Vec3D.spherical(r,np.radians(theta),np.radians(rho))

    @staticmethod
    def rodrigues(v1,v2,theta):
        """
        Rodrigues' rotation formula

        https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
        
        Args:
            v1 (mObjects.Vec3D) : vector to be modified
            v1 (mObjects.Vec3D) : vector to be rotated around
            theta (float) : rotation angle in rads
        
        Returns:
            mObjects.Vec3D : rotated vector
        """
        kvec = Vec3D.normalize(Vec3D.cross(v1,v2))
        return v1*np.cos(theta) + Vec3D.cross(kvec,v1)*np.sin(theta) + kvec*Vec3D.dot(kvec,v1)*(1-np.cos(theta))

# root object
class SimObject:
    # position(pVec), velocity(vVec), acceleration(aVec), frontwards(fVec), data dictionary(dataDict)
    def __init__(self, pVec=None, vVec=None, aVec=None, fVec = None, data=None):
        self.pVec = Vec3D() if pVec == None else pVec
        self.vVec = Vec3D(1,0,0) if vVec == None else vVec
        self.aVec = Vec3D() if aVec == None else aVec
        self.fVec = Vec3D(1,0,0) if fVec == None else Vec3D.normalize(fVec)
        self.upVec = Vec3D(0,0,1)
        self.data = {} if data == None else data

        self.bank = 0 # Use as rightVec

        #self.aileron = 0 #0, %
        #self.elevator = 0 #0, % 
        #self.rudder = 0 #-7, % 
        #self.Ny = 0 #1 #GForce
        #self.Vy = 0 #0, m/s #Upwards Velocity
        self.Wx = 0 #0, deg/s #Rolling
        self.AoA = 0 #3.9, deg
        self.AoS = 0 #72.5, deg
        self.IAS = 0 #0, km/h
        
        #----------------
        #self.speed = 0 #-0.0281
        #self.bank = 0 # 0.056
        self.turn = 0 # 0
        #self.head_temperature = 0 #235.59
        #self.trimmer = 0 #0

# (1x SimObject), 1x fireFlareAt[float,...], 1x isAfterburnerOnAt[[float,float],...]
class TargetObject(SimObject):
    def __init__(self, pVec=None, vVec=None, aVec=None, fVec = None, data=None, fireFlareAt=None, isAfterburnerOnAt=None):
        super().__init__(pVec,vVec,aVec,fVec,data)
        self.fireFlareAt = [] if fireFlareAt == None else fireFlareAt
        self.isAfterburnerOnAt = [] if isAfterburnerOnAt == None else isAfterburnerOnAt
        self.isAfterburnerOn = False
        self.isHit = False

# same as SimObject?
class FlareObject(SimObject):
    def __init__(self, pVec=None, vVec=None, aVec=None, fVec = None, data=None, firedAt=None):
        newpVec = Vec3D(INFINITE,INFINITE,INFINITE) if pVec == None else pVec
        super().__init__(newpVec,vVec,aVec,fVec,data)
        self.isFired = False
        self.isOff = False

# (1x SimObject), 1x seekerVec, booleans...
class MissileObject(SimObject):
    # ...seeker direction(sVec), acceleration(aVec)
    def __init__(self, pVec=None, vVec=None, aVec=None, fVec=None, sVec=None, data=None):
        super().__init__(pVec,vVec,aVec,fVec,data)
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