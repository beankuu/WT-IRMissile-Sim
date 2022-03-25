import math
import numpy as np

#customs
import data
import calculate as calc
import mObjects as obj

#!===============================================================
# Guidance CALCULATIONS
#!===============================================================

#band 1(engine)
engineTemp = data.t['Thrust'] * data.thrustKgsToInfraRedBrightness
afEngineTemp = engineTemp + (data.t['AfterburnerBoost'] * data.t['ThrustMult'] -1)* data.t['Thrust'] * data.afterburnerThrustKgsToInfraRedBrightness

#band 2(airframe)
def getMissileLocation(time):
    pass

#!===============================================================
# Force CALCULATIONS
#!===============================================================
## ??

wingArea = math.pi *math.pow((data.m['caliber'])/2,2)*data.m['wingAreamult']

Thrust = data.m['force'] * data.m['timeFire']