# bunch of hardcoded datas + helper functions

# missile1 : sidewinder AIM-9D
missile1 = {
    'name' : 'AIM-9D',
    'caliber' : 0.127,    'mass' : 88.45,   'massEnd' : 55.8,    'dragCx' : 0.018,    'length' : 3.0,    'wingAreamult' : 1.4,    'distFromCmToStab' : 0.01,    
    'CxK' : 4.2,    'finsAoA' : 0.18,    'force' : 13070,    'timeFire' : 5.0,    'startSpeed': 0,    'endSpeed' : 1000,    'timeLife' : 60.0,    
    'machMax' : 2.5,    'loadFactorMax' : 18.0,    'rangeMax' : 18000.0,
    'g_fov' : 2.5,    'g_lockAngleMax' : 10.0,    'g_angleMax' : 40.0,    'g_rateMax' : 12.0, 
    'g_ga_propNavMult' : 4.0,    'g_ga_reqAccelMax' : 18.0,    'g_ga_baseIndSpeed' : 1800.0,
    'g_ga_accelControlProp' : 0.01,    'g_ga_accelControlIntg' : 0.005,    'g_ga_accelControlIntgLim' : 0.75,    'g_ga_accelControlDiff' : 0.001,
    't0_altitude' : 0.0, 't0_figherMach' : [1.2,0.8], 't0_targetMach' : [0.8,0.8], 't0_rangeMin' : [500,3000,400,2100], 't0_rangeMax' : [3400,4300,2400,2750], 't0_altdiff' : [500,1000],
    't1_altitude' : 6000.0, 't1_figherMach' : [1.2,0.8], 't1_targetMach' : [0.8,0.8], 't1_rangeMin' : [600,3000,500,2100], 't1_rangeMax' : [5400,8500,4800,6400], 't1_altdiff' : [500,1000]
}
# missile2 : R-60
missile2 = {
    'name' : 'R-60',
    'caliber' : 0.12,    'mass' : 44.0,   'massEnd' : 34.0,    'dragCx' : 0.015,    'length' : 2.1,    'wingAreamult' : 1.25,    'distFromCmToStab' : 0.06,    
    'CxK' : 3.1,    'finsAoA' : 0.18,    'force' : 9500,    'timeFire' : 3.0,    'startSpeed': 0,    'endSpeed' : 0,    'timeLife' : 21.0,    
    'machMax' : 2.5,    'loadFactorMax' : 30.0,    'rangeMax' : 8000.0,
    'g_fov' : 5.0,    'g_lockAngleMax' : 12.0,    'g_angleMax' : 45.0,    'g_rateMax' : 35.0,
    'g_ga_propNavMult' : 4.0,    'g_ga_reqAccelMax' : 30.0,    'g_ga_baseIndSpeed' : 1700.0,
    'g_ga_accelControlProp' : 0.01,    'g_ga_accelControlIntg' : 0.005,    'g_ga_accelControlIntgLim' : 0.5,    'g_ga_accelControlDiff' : 0.001,
    't0_altitude' : 1000.0, 't0_figherMach' : [0.91,0.74], 't0_targetMach' : [0.74,0.58], 't0_rangeMin' : [300,2600,250,1700], 't0_rangeMax' : [2300,3600,2500,3800], 't0_altdiff' : [500,1000],
    't1_altitude' : 5000.0, 't1_figherMach' : [0.95,0.61], 't1_targetMach' : [0.61,0.78], 't1_rangeMin' : [300,2600,250,1700], 't1_rangeMax' : [3900,5000,4300,5500], 't1_altdiff' : [500,1000]
}
#*------------------------------------------------------------------------------------
# thrust, afterburner thrust, enginecount
# F-4E
target1 = {
    'name' : 'F-4E',
    'mass' : 20000, #kg
    'maxspeed' : 370, #m/s (1332kmh)
    'Thrust' : 2600,
    'AfterburnerBoost' : 1.1,
    'ThrustMult' : 1.4,
    'enginecount' : 2,
    'flareBrightness' : 1000.0,
    'flareLiveTime' : 4.4
}
# mig-23MLD
target2 = {
    'name' : 'Mig-23MLD',
    'mass' : 14000, #kg,
    'maxspeed' : 370, #m/s (1332kmh)
    'thrust' : 6057.0601,
    'AfterburnerBoost' : 1.1,
    'ThrustMult' : 1.32,
    'enginecount' : 1,
    'flareBrightness' : 4500.0,
    'flareLiveTime' : 4.4
}
#*------------------------------------------------------------------------------------
thrustKgsToInfraRedBrightness = 1.0
afterburnerThrustKgsToInfraRedBrightness = 4.5
EngineIRMultFront = 0.0006
EngineIRMultSide  = 0.06
EngineIRMultRear = 1.0
"""
AFIRMult = 20
AFIRTempMach = 0.8
AFIRTemp2BrightPwr = 10.0
AFIRTemp2BrightMult0 = [0, 1]
AFIRTemp2BrightMult1 = [5000, 2.4]
"""
"""
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
#*------------------------------------------------------------------------------------
m = missile1
t = target1

INIT_MISSILE_POSITION=[0,0,1500]   # m
#horizontal coordinate(r,inclination,azimuth), degree
INIT_TARGET_HORIZ_COORDINATE=(2000, 0, 20)
TARGET_INIT_VELOCITY = 280 #(m/s), 1008kmh

## need for every 100ms 
## sample of dots
RANGEMAX = 5000
RANGE_HOP = 500
TIMEMAX = 6
TIME_HOP = 1/m['g_rateMax']
#30fps?
FPS = 30
MaxFrame = TIMEMAX*FPS


#!===============================================================
# other CALCULATIONS
#!===============================================================
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
def getLinear(lst,height):
    index = min(range(len(lst)),key=lambda i: abs(lst[i][0]-height))
    if index == 0 : index += 1
    aH = lst[index][0];   aV = lst[index][1]
    bH = lst[index-1][0]; bV = lst[index-1][1]
    return bV + (height-bH)*((bV-aV)/(bH-aH))
# height(m)
def getAirDensity(height):
    return round(getLinear(airDensityList,height),6)

# acceleration(m/s^2) height(m)
def getGravity(height):
    return round(getLinear(gravityList,height),4)
def getGForce(acceleration, height):
    return round(acceleration / getGravity(height),1)

# height(m) temperature(C)
def getTemperatureAtHeight(height):
    return round(getLinear(temperatureList,height),3)
# velocity(m/s), height(m)
def getMach(velocity, height):
    #http://www.aerospaceweb.org/question/atmosphere/q0126.shtml
    # mach = velocity / sound_speed_at_given_altitude
    # sound_speed_at_given_altitude = 331 + 0.6*temperature
    speed_of_sound = 331 + 0.6*getTemperatureAtHeight(height)
    return round(velocity/speed_of_sound,2)