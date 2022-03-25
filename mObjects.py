# dot, cross, norm
import numpy as np

class Vec3D:
    # P = list(3) or Vec3D(3) ...
    def __init__(self, x = None,y = None,z = None):
        self.x = 0 if x == None else x
        self.y = 0 if y == None else y
        self.z = 0 if z == None else z
    # +
    def __add__(self, other):
        if isinstance(other, Vec3D):
            return Vec3D(self.x + other.x, self.y + other.y, self.z + other.z)
        elif type(other) is int or type(other) is float:
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
        if isinstance(other, Vec3D):
            return Vec3D(self.x - other.x, self.y - other.y, self.z - other.z)
        elif type(other) is int or type(other) is float:
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
        if type(other) is not int and type(other) is not float:
            raise TypeError('Vec3D mul only supports scalar(int or float) operations')
        return Vec3D(self.x * other, self.y * other, self.z * other)
    def __imul__(self,other):
        return self * other
    def __rmul__(self, other):
        return self * other
    # / scalar only, no floordiv!
    def __truediv__(self, other):
        if type(other) is not int and type(other) is not float:
            raise TypeError('Vec3D mul only supports scalar(int or float) operations')
        # some big number
        divider = 2147483648 if other == 0 else other
        return Vec3D(self.x/divider, self.y/other, self.z/other)
    def __itruediv__(self,other):
        return self / other
    # ==
    def __eq__(self, other):
        if isinstance(other, Vec3D):
            return self.x == other.x and self.y == other.y and self.z == other.z
        else:
            return self.x == other[0] and self.y == other[1] and self.z == other[2]
    def __req__(self,other):
        return self == other

    #---------------------------------
    # degree in rads, axis = 'x','y','z'
    def rotate(self, degree, axis):
        rot_cos = np.cos(degree)
        rot_sin = np.cos(degree)
        if axis == 'x':
            #self.x = self.x
            self.y = rot_cos*self.y - rot_sin*self.z
            self.z = rot_sin*self.y + rot_cos*self.z
        elif axis == 'y':
            self.x = rot_cos*self.x + rot_sin*self.z
            #self.y = self.y
            self.z = -rot_sin*self.x + rot_cos*self.z
        else:
            self.x = rot_cos*self.x - rot_sin*self.y
            self.y = rot_sin*self.x + rot_cos*self.y
            #self.z = self.z
        return self #chaining
    def translate(self, P):
        return self + P #chaining
    def scale(self, P):
        return self * P #chaining
    def dot(self, other):
        return np.dot([self.x,self.y,self.z],[other.x,other.y,other.z])
    def cross(self, other):
        result = np.cross([self.x,self.y,self.z],[other.x,other.y,other.z])
        self.x = result[0]; self.y = result[1]; self.z = result[2]
        return self #chaining
    def normalize(self):
        return self / float(np.linalg.norm(np.array([self.x,self.y,self.z]))) #chaining

    # to be used...?
    def setSpherical(self,r,rho,theta):
        self.x = r*np.cos(rho)*np.sin(theta)
        self.y = r*np.sin(rho)*np.sin(theta)
        self.z = r*np.cos(rho)
        return self #chaining

    def toList(self):
        return [self.x, self.y, self.z]
    def __str__(self):
        return [self.x, self.y, self.z]
    def __repr__(self):
        return [self.x, self.y, self.z]

# 1x posVec, 1x dirVec
class SimObject:
    def __init__(self, pVec=None, dVec=None):
        self.pVec = Vec3D() if pVec == None else pVec
        self.dVec = Vec3D() if dVec == None else dVec
    def rotate(self,degree,axis):
        self.pVec = self.pVec.rotate(degree,axis)
        self.dVec = self.dVec.rotate(degree,axis)
        return self
    
# 1x posVec, 1x dirVec
class TargetObject(SimObject):
    def __init__(self):
        super().__init__(self)

# (1x posVec, 1x dirVec), 1x seekerVec
class MissileObject(SimObject):
    def __init__(self):
        super().__init__(self)
        self.sVec = Vec3D()
    def rotate(self,degree,axis):
        super().rotate(degree,axis)
        self.sVec = self.sVec.rotate(degree,axis)
        return self