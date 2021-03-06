# bunch of hardcoded datas + helper functions

#!===================================================
# Global constants
INFINITE = 2147483647

TIMEMAX = 10
FPS = 30
MaxFrame = TIMEMAX*FPS
dt = 1/FPS

#----------

horsePowersToInfraRedBrightness: 0.42
thrustKgsToInfraRedBrightness = 1.0
afterburnerThrustKgsToInfraRedBrightness = 4.5
EngineIRMultFront = 0.0006
EngineIRMultSide  = 0.06
EngineIRMultRear = 1.0

"""
#----------
## snow
w_snow = -250
## Clear state
w_clear = -150
## cloudy
w_cloudy = 0
## Rain
w_rain = 400
## Storm
w_storm = 600

#envAddWhiteTemp = w_cloudy
"""


#!===================================================
# missile0 : AIM-9B
missile0 = {
    'name' : 'AIM-9B',
    'caliber' : 0.127,    'mass' : 72.57,   'massEnd' : 55.8,    'dragCx' : 0.018,    'length' : 2.83,    'wingAreamult' : 1.4,    'distFromCmToStab' : 0.1,    
    'CxK' : 3.3,    'finsAoA' : 0.16,    'force' : 17263.0,    'timeFire' : 2.2,     'endSpeed' : 1000.0,    'timeLife' : 20.0,    
    'machMax' : 1.7,    'loadFactorMax' : 10.0,    'rangeBand0' : 4000.0,
    'g_fov' : 4.0,      'g_lockAngleMax' : 5.0,    'g_angleMax' : 25.0,    'g_rateMax' : 11.0,
    'g_ga_timeOut' : 0.5,  'g_ga_propNavMult' : 4.0,    'g_ga_reqAccelMax' : 11.0,    'g_ga_baseIndSpeed' : 1800.0,
    'g_ga_accelControlProp' : 0.01,    'g_ga_accelControlIntg' : 0.005,    'g_ga_accelControlIntgLim' : 0.75,    'g_ga_accelControlDiff' : 0.001,
    'proximityFuse_radius' : 5.0
}
# missile1 : sidewinder AIM-9D
missile1 = {
    'name' : 'AIM-9D',
    'caliber' : 0.127,    'mass' : 88.45,   'massEnd' : 55.8,    'dragCx' : 0.018,    'length' : 3.0,    'wingAreamult' : 1.4,    'distFromCmToStab' : 0.01,    
    'CxK' : 4.2,    'finsAoA' : 0.18,    'force' : 13070,    'timeFire' : 5.0,  'endSpeed' : 1000,    'timeLife' : 60.0,    
    'machMax' : 2.5,    'loadFactorMax' : 18.0,    'rangeBand0' : 5500.0,
    'g_fov' : 2.5,    'g_lockAngleMax' : 10.0,    'g_angleMax' : 40.0,    'g_rateMax' : 12.0, 
    'g_ga_timeOut' : 0.5,  'g_ga_propNavMult' : 4.0,    'g_ga_reqAccelMax' : 18.0,    'g_ga_baseIndSpeed' : 1800.0,
    'g_ga_accelControlProp' : 0.01,    'g_ga_accelControlIntg' : 0.005,    'g_ga_accelControlIntgLim' : 0.75,    'g_ga_accelControlDiff' : 0.001,
    'proximityFuse_radius' : 5.0

}
# missile2 : R-60
missile2 = {
    'name' : 'R-60',
    'caliber' : 0.12,    'mass' : 44.0,   'massEnd' : 34.0,    'dragCx' : 0.015,    'length' : 2.1,    'wingAreamult' : 1.25,    'distFromCmToStab' : 0.06,    
    'CxK' : 3.1,    'finsAoA' : 0.18,    'force' : 9500,    'timeFire' : 3.0,   'endSpeed' : 0,    'timeLife' : 21.0,    
    'machMax' : 2.5,    'loadFactorMax' : 30.0,    'rangeBand0' : 5000.0,
    'g_ga_timeOut' : 0.35,  'g_fov' : 5.0,    'g_lockAngleMax' : 12.0,    'g_angleMax' : 45.0,    'g_rateMax' : 35.0,
    'g_ga_propNavMult' : 4.0,    'g_ga_reqAccelMax' : 30.0,    'g_ga_baseIndSpeed' : 1700.0,
    'g_ga_accelControlProp' : 0.01,    'g_ga_accelControlIntg' : 0.005,    'g_ga_accelControlIntgLim' : 0.5,    'g_ga_accelControlDiff' : 0.001,
    'proximityFuse_radius' : 3.0
}
# missile3 : PL-5B
missile3 = {
    'name' : 'PL-5B',
    'caliber' : 0.127,    'mass' : 84.0,   'massEnd' : 59.7,    'dragCx' : 0.018,    'length' : 2.1,    'wingAreamult' : 1.4,    'distFromCmToStab' : 0.1,    
    'CxK' : 3.3,    'finsAoA' : 0.18,    'force' : 36000,    'timeFire' : 2.0,     'endSpeed' : 1000.0,    'timeLife' : 40.0,    
    'machMax' : 2.2,    'loadFactorMax' : 30.0,    'rangeBand0' : 5500.0,
    'g_fov' : 2.4,    'g_lockAngleMax' : 25.0,    'g_angleMax' : 40.0,    'g_rateMax' : 20.0,
    'g_ga_timeOut' : 0.5,  'g_ga_propNavMult' : 4.0,    'g_ga_reqAccelMax' : 30.0,    'g_ga_baseIndSpeed' : 1800.0,
    'g_ga_accelControlProp' : 0.01,    'g_ga_accelControlIntg' : 0.005,    'g_ga_accelControlIntgLim' : 0.75,    'g_ga_accelControlDiff' : 0.002,
    'proximityFuse_radius' : 5.0
}
# missile4 : AIM-9L
missile4 = {
    'name' : 'AIM-9L',
    'caliber' : 0.127,    'mass' : 84.46,   'massEnd' : 57.06,    'dragCx' : 0.018,    'length' : 2.85,    'wingAreamult' : 1.4,    'distFromCmToStab' : 0.1,    
    'CxK' : 4.1,    'finsAoA' : 0.2,    'force' : 12000,    'timeFire' : 5.3,     'endSpeed' : 1000.0,    'timeLife' : 60.0,    
    'machMax' : 2.5,    'loadFactorMax' : 30.0,    'rangeBand0' : 6000.0,
    'g_fov' : 2.5,    'g_lockAngleMax' : 17.0,    'g_angleMax' : 45.0,    'g_rateMax' : 35.0,
    'g_ga_timeOut' : 0.5, 'g_ga_propNavMult' : 4.0,    'g_ga_reqAccelMax' : 30.0,    'g_ga_baseIndSpeed' : 1440.0,
    'g_ga_accelControlProp' : 0.01,    'g_ga_accelControlIntg' : 0.005,    'g_ga_accelControlIntgLim' : 0.75,    'g_ga_accelControlDiff' : 0.003,
    'proximityFuse_radius' : 5.0
}
# missile5 : AIM-9J
missile5 = {
    'name' : 'AIM-9J',
    'caliber' : 0.127,    'mass' : 76.03,   'massEnd' : 58.0,    'dragCx' : 0.018,    'length' : 3.05,    'wingAreamult' : 1.4,    'distFromCmToStab' : 0.1,    
    'CxK' : 3.3,    'finsAoA' : 0.18,    'force' : 18100,    'timeFire' : 2.2,     'endSpeed' : 1000.0,    'timeLife' : 40.0,    
    'machMax' : 2.5,    'loadFactorMax' : 20.0,    'rangeBand0' : 5500.0,
    'g_fov' : 2.5,    'g_lockAngleMax' : 10.0,    'g_angleMax' : 40.0,    'g_rateMax' : 16.5,
    'g_ga_timeOut' : 0.5, 'g_ga_propNavMult' : 4.0,    'g_ga_reqAccelMax' : 20.0,    'g_ga_baseIndSpeed' : 1620.0,
    'g_ga_accelControlProp' : 0.01,    'g_ga_accelControlIntg' : 0.005,    'g_ga_accelControlIntgLim' : 0.5,    'g_ga_accelControlDiff' : 0.001,
    'proximityFuse_radius' : 5.0
}
# missile6 : R-24T
missile6 = {
    'name' : 'R-24T',
    'caliber' : 0.2,    'mass' : 237.0,   'massEnd' : 162.0,    'dragCx' : 0.018,    'length' : 4.194,    'wingAreamult' : 1.35,    'distFromCmToStab' : 0.06,    
    'CxK' : 2.1,    'finsAoA' : 0.12,    'force' : 50000,    'timeFire' : 3.0,     'endSpeed' : 1500.0,    'timeLife' : 45.0,    
    'machMax' : 3.5,    'loadFactorMax' : 24.0,    'rangeBand0' : 25000.0,
    'g_fov' : 2.6,    'g_lockAngleMax' : 55.0,    'g_angleMax' : 55.0,    'g_rateMax' : 20.0,
    'g_ga_timeOut' : 0.0, 'g_ga_propNavMult' : 4.0,    'g_ga_reqAccelMax' : 24.0,    'g_ga_baseIndSpeed' : 1580.0,
    'g_ga_accelControlProp' : 0.04,    'g_ga_accelControlIntg' : 0.02,    'g_ga_accelControlIntgLim' : 0.8,    'g_ga_accelControlDiff' : 0.002,
    'proximityFuse_radius' : 10.0
}
#!------------------------------------------------------------------------------------
# thrust, afterburner thrust, enginecount
target0 = {
    'name' : 'Target',
    'mass' : 1000, #kg
    'maxspeed' : 350, #m/s (1332kmh)
    'Thrust' : 5250,
    'AoA' : 0.25, #90 deg
    'AfterburnerBoost' : 1.1,
    'ThrustMult' : 1.4,
    'enginecount' : 2,
    'flareType' : 0
}
# F-4E
target1 = {
    'name' : 'F-4E',
    'mass' : 20000, #kg
    'maxspeed' : 350, #m/s (1260kmh)
    'AoA' : 0.138, #50 deg,
    'wingAreaSum' : 8.2*6+4.0+5.2+1.8+5.0+1.5+0.35*2,
    'Thrust' : 5250,
    'AfterburnerBoost' : 1.1,
    'ThrustMult' : 1.4,
    'enginecount' : 2,
    'flareType' : 0
}
# mig-23MLD
target2 = {
    'name' : 'Mig-23MLD',
    'mass' : 14000, #kg,
    'maxspeed' : 360, #m/s (1296kmh)
    'AoA' : 0.25, #90 deg
    'wingAreaSum' : (4+7+6+4+7+6+0.2)*7.8,
    'Thrust' : 7900,
    'AfterburnerBoost' : 1.1,
    'ThrustMult' : 1.32,
    'enginecount' : 1,
    'flareType' : 1
}
# A-10a early
target3 = {
    'name' : 'A-10A',
    'mass' : 7000, #kg,
    'maxspeed' : 140, #m/s (504kmh)
    'stallSpeed' : 49, #m/s (176.4kmh)
    'AoA' : 0.138, #50 deg,
    'wingAreaSum' : (7.8*6+2.05)*17.5,
    'Thrust' : 9.80665*1981.96,
    'AfterburnerBoost' : 1.0,
    'ThrustMult' : 1.0,
    'enginecount' : 2,
    'flareType' : 0
}
#!------------------------------------------------------------------------------------
flare0 = {
    'name' : 'flare',
    'flareBrightness' : 1000.0,
    'timeLife' : 4.4, 'timeFire' : 4.4,
    'caliber' : 0.026,
    'mass' : 0.09, 'massEnd' : 0.04,
    'maxDeltaAngle' : 25.0, 'maxDeltaAngleVertical' : 5.0,
    'dragCx' : 0.0001, 'CxK' : 8.0,
    'length' : 0.008, 'distFromCmToStab' : 0.005,
    'force' : 0.1,
    'startSpeed' : 70.0, 'endSpeed' : 0.0
}
flare1 = {
    'name' : 'flare_big',
    'flareBrightness' : 4500.0,
    'timeLife' : 4.5, 'timeFire' : 4.4,
    'caliber' : 0.05,
    'mass' : 1.0, 'massEnd' : 0.27,
    'maxDeltaAngle' : 25.0, 'maxDeltaAngleVertical' : 5.0,
    'dragCx' : 0.0001, 'CxK' : 8.0,
    'length' : 0.2, 'distFromCmToStab' : 0.005,
    'force' : 0.1,
    'startSpeed' : 70.0, 'endSpeed' : 0.0
}

#!===============================================================
# other Datas and calculations
#----------------------------------------------------------------
#https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
# Air Density table
airDensityList = [
    (-1000, 1.347), (0, 1.225), 
    (1000, 1.112),  (2000, 1.007), (3000, 0.9093), (4000, 0.8194), (5000, 0.7364),
    (6000, 0.6601), (7000, 0.5900), (8000, 0.5258), (9000, 0.4671), (10000, 0.4135),  
    (15000, 0.1948), (20000, 0.08891), (25000,0.04008),  (30000, 0.01841), (40000,0.003996),
    (50000, 0.001027), (60000,0.0003097), (70000,0.00008283), (80000,0.00001846)
]
# Gravity table
gravityList = [
    (-1000, 9.810), (0, 9.807), 
    (1000, 9.804), (2000, 9.801), (3000, 9.797), (4000, 9.794), (5000, 9.791),
    (6000, 9.788), (7000, 9.785), (8000, 9.782), (9000, 9.779), (10000, 9.776),  
    (15000, 9.761), (20000, 9.745), (25000, 9.730), (30000, 9.715), (40000,9.684),
    (50000, 9.654), (60000, 9.624), (70000,9.594), (80000, 9.564)
]
# Atmosphere Temperature table
temperatureList = [
    (-1000, 21.50), (0, 15.00), 
    (1000, 8.50), (2000, 2.00), (3000,-4.49), (4000, -10.98), (5000, -17.47),
    (6000, -23.96), (7000, -30.45), (8000, -36.94), (9000, -43.42), (10000,-49.90),  
    (15000, -56.50), (20000, -56.50), (25000,-51.60),  (30000, -46.64),  (40000, -22.80),
    (50000, -2.5), (60000,-26.13), (70000,-53.57), (80000, -74.51)
]
def calcLinear(pairsList,reqKey):
    """
    linearly calculates 'value' for given 'key', from given 'list of pairs'

    Args:
        pairsList (int): list of pairs(key,value)
        reqKey    (str): key, for wanted value

    Returns:
        float: linearly calculated value
    """
    index = min(range(len(pairsList)),key=lambda i: abs(pairsList[i][0]-reqKey))
    if index == 0 : index += 1
    aH = pairsList[index][0];   aV = pairsList[index][1]
    bH = pairsList[index-1][0]; bV = pairsList[index-1][1]
    return bV + (reqKey-bH)*((bV-aV)/(bH-aH))

def getAirDensity(height):
    """
    gets density of air, at given height

    Args:
        height (float): height of object(m)

    Returns:
        float: density of air
    """
    return calcLinear(airDensityList,height)

def getGravity(height):
    """
    gets gravity, at given height

    Args:
        height (float): height of object(m)

    Returns:
        float: gravity(m/s^2)
    """
    return calcLinear(gravityList,height)

def getTemperatureAtHeight(height):
    """
    gets environment temperature, at given height

    Args:
        height (float): height of object(m)

    Returns:
        float: environment temperature(C)
    """
    return calcLinear(temperatureList,height)

def calcMach(velocity, height):
    """
    gets Mach speed, at given height

    Args:
        velocity (float):   velocity of object(m/s)
        height (float):     height of object(m)

    Returns:
        float: mach
    """
    #http://www.aerospaceweb.org/question/atmosphere/q0126.shtml
    # mach = velocity / sound_speed_at_given_altitude
    # sound_speed_at_given_altitude = 331 + 0.6*temperature
    speed_of_sound = 331 + 0.6*getTemperatureAtHeight(height)
    return velocity/speed_of_sound