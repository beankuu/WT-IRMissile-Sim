import pytest

# to test
from mObjects import Vec3D as vec3
import numpy as np

#* mObjects
# vec3D
def test_Vec3D():
    obj1 = vec3(3,0,9)
    obj2 = vec3(1,0,-1)
    print("Hello")
    # __init__
    assert obj1.x == 3 and obj1.y == 0 and obj1.z == 9
    #-------------
    # arithmetic (+ - * / == =) [except iadd, isub, imul, itruediv]
    # add (add[Vec3D], radd, pos)
    assert (obj1 + obj2) == vec3(4,0,8)
    assert (obj2 + obj1) == vec3(4,0,8)
    assert (+obj1) == vec3(3,0,9)
    # sub (sub[Vec3D] rsub(incl. neg))
    assert (obj1 - obj2) == vec3(2,0,10)
    assert (obj2 - obj1) == vec3(-2,0,-10)
    assert -(obj1 - obj2) == vec3(-2,0,-10)
    # mul ([int&float]) (mul, rmul)
    assert (obj1*2) == vec3(6,0,18)
    assert (-3*obj1) == vec3(-9,0,-27)
    # div ([int&float]) (truediv)
    assert (obj1/3) == vec3(1,0,3)
    # iadd,isub,imul,itruediv
    obj1 += vec3(0,-2,0)
    assert obj1 == vec3(3,-2,9)
    obj1 -= vec3(3,0,1)
    assert obj1 == vec3(0,-2,8)
    obj1 *= -2
    assert obj1 == vec3(0,4,-16)
    obj1 /= 4
    assert obj1 == vec3(0,1,-4)
    # ==
    # Used in tests above!
    #-------------
    # __str__(self)
    # __repr__(self)
    #-------------
    assert (type(obj1)) == type(vec3())
    # =
    obj2 = obj1
    assert obj2 == vec3(0,1,-4)
    assert obj2 != vec3(0,0,0)
    #-------------
    # clone(self) #!
    obj1 = vec3(1,1,1)
    obj2 = obj1#.clone()
    obj3 = obj1
    obj3 += vec3(3,0,0)
    assert obj1 == obj2 #!
    assert obj1 != obj3 #!
    # toArray(self)
    assert np.array_equal(obj2.toArray(),np.array([1,1,1]))
    assert not np.array_equal(obj2.toArray(),np.array([1,0,1]))
    # toList(self)
    assert obj2.toList() == vec3(1,1,1)
    assert obj2.toList() != vec3(1,0,1)
    #-------------
    # static functions
    # rotate(obj,rads,axis)
    obj1 = vec3.rotate(vec3(0,1,0),np.pi/2,'x')
    assert pytest.approx(obj1.x, 0.0001) == 0
    assert pytest.approx(obj1.y, 0.0001) == 0
    assert pytest.approx(obj1.z, 0.0001) == 1

    # rotateDegree(obj,degree,axis)
    obj1 = vec3.rotateDegree(vec3(1,0,0),90,'z')
    assert pytest.approx(obj1.x, 0.0001) == 0
    assert pytest.approx(obj1.y, 0.0001) == 1
    assert pytest.approx(obj1.z, 0.0001) == 0

    obj1 = vec3.rotateDegree(vec3(0,0,1),90,'y')
    assert pytest.approx(obj1.x, 0.0001) == 1
    assert pytest.approx(obj1.y, 0.0001) == 0
    assert pytest.approx(obj1.z, 0.0001) == 0

    # translate
    assert vec3.translate(vec3(1,0,0), vec3(3,1,-1)) == vec3(4,1,-1)

    # scale
    assert vec3.scale(vec3(1,0,0), 3) == vec3(3,0,0)

    # dot
    assert vec3.cross(vec3(1,0,0), vec3(0,1,0)) == vec3(0,0,1)

    # cross
    assert vec3.cross(vec3(1,0,0), vec3(0,1,0)) == vec3(0,0,1)
    assert vec3.cross(vec3(0,1,0), vec3(1,0,0)) == vec3(0,0,-1)

    # norm
    ## Pythagorean triples!
    assert vec3.norm(vec3(3,4,0)) == 5
    assert vec3.norm(vec3(3,4,1)) != 5

    # normalize
    ## Pythagorean triples!
    assert vec3.normalize(vec3(3,0,4)) == vec3(3/5,0,4/5)
    assert vec3.normalize(vec3(3,1,4)) != vec3(3/5,0,4/5)
    
    # spherical(r,theta,rho)
    obj1 = vec3.spherical(10,0,np.pi/2)
    assert pytest.approx(obj1.x, 0.0001) == 0
    assert pytest.approx(obj1.y, 0.0001) == 0
    assert pytest.approx(obj1.z, 0.0001) == 10
    # sphericalDegree(r,theta,rho)
    obj1 = vec3.sphericalDegree(*[10,90,0])
    assert pytest.approx(obj1.x, 0.0001) == 10
    assert pytest.approx(obj1.y, 0.0001) == 0
    assert pytest.approx(obj1.z, 0.0001) == 0
