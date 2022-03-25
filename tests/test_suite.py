import pytest

# to test
import mObjects

#* mObjects
# vec3D
def test_Vec3D():
    obj1 = mObjects.Vec3D(3,0,9)
    obj2 = mObjects.Vec3D(1,0,-1)
    print("Hello")
    # __init__
    assert obj1.x == 3 and obj1.y == 0 and obj1.z == 9
    # arithmetic (+ - * / == =) [except iadd, isub, imul, itruediv]
    # add (add[Vec3D, int&float, other], radd, pos)
    assert (obj1 + obj2) == [4,0,8]
    assert (obj2 + obj1) == [4,0,8]
    assert (obj1 + [0,0,1]) == [3,0,10]
    assert ([0,0,1] + obj1) == [3,0,10]
    assert (obj1 + 3) == [6,3,12]
    assert (3 + obj1) == [6,3,12]
    assert (+obj1) == [3,0,9]
    # sub (sub[Vec3D, int&float, other], rsub(incl. neg))
    assert (obj1 - obj2) == [2,0,10]
    assert (obj2 - obj1) == [-2,0,-10]
    assert (obj1 - [0,0,1]) == [3,0,8]
    assert ([0,0,1] - obj1) == [-3,0,-8]
    assert (obj1 - 3) == [0,-3,6]
    assert (3 - obj1) == [0,3,-6]
    # mul ([int&float]) (mul, rmul)
    assert (obj1*2) == [6,0,18]
    assert (-3*obj1) == [-9,0,-27]
    # div ([int&float]) (truediv)
    assert (obj1/3) == [1,0,3]
    # iadd,isub,imul,itruediv
    obj1 += [0,-2,0]
    assert obj1 == [3,-2,9]
    obj1 -= [3,0,1]
    assert obj1 == [0,-2,8]
    obj1 *= -2
    assert obj1 == [0,4,-16]
    obj1 /= 4
    assert obj1 == [0,1,-4]
    # ==
    # Used in tests above!
    assert (type(obj1)) == type(mObjects.Vec3D())
    # =
    obj2 = obj1
    assert obj2 == [0,1,-4]
    # functions(rotate, translate, scale, dot, cross, normalize)
    
#SimObject
def test_SimObject():
    # __init__
    obj1 = mObjects.SimObject()
    # functions(rotate)

def test_TargetObject():
    # __init__
    obj1 = mObjects.TargetObject()
    
def test_MissileObject():
    obj1 = mObjects.MissileObject()
    # functions(rotate)
