import math
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.animation as ani
from mpl_toolkits.mplot3d import Axes3D


#============================
#============================
#============================
"""
$ PN 
* ASSUME Relative velocity(Vr) == speed of rocket
@ IN_FIXED = N
@ IN_CALCULATED = ohm, Vr
@ OUT = an

an = N*Ohm*Vr
or
a_vec = -N * |Vr| * vm_vec / |vm_vec| * ( (R_vec x Vr_vec) / (R_vec*R_vec) )

"""
#----------------------------
"""
$ PID
@ IN_FIXED = Kp, KI, KILim, KD
@ IN_CALCULATED = an-1
@ OUT = an

"""
#----------------------------
"""
$ Lift vs Gravity, Drag vs Thrust
@ IN_FIXED = CdK, Cx, mass, WingMaxAoA, gravity?
@ IN_CALCULATED = speed, airDensity, WingAoA
@ OUT = a


Drag(D) = Coeff(Cd) * (air)Density * velocity^2 * (wing)Area * 0.5
Lift(L) = Coeff(CL) * (air)Density * velocity^2 * (wing)Area * 0.5

CDrag = CdK + K(CL - CL0)^2,
where CdK = K*CL^2, (given)CxK = Drag lift-induced-drag (K)

CLMax = 2 * PI * CritAoA(rad)

"""
#----------------------------
#============================
#----------------------------
"""
$ Basic Approach

acceleration request = PN & PID
Range = Thrust & Lift & Drag

LIMITS? 
- [PID] KILim
- maxMach
- GMax
- startv
- endv
- baseIndSpeed

"""
#============================
#============================
#============================

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

INIT_MISSILE_POSITION=np.array([0,0,1500])   # m
#horizontal coordinate(r,inclination,azimuth), rad
INIT_TARGET_HORIZ_COORDINATE=(2000,0,np.radians(20))
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
# Guidance CALCULATIONS
#!===============================================================
#band 1(engine)
engineTemp = t['Thrust'] * thrustKgsToInfraRedBrightness
afEngineTemp = engineTemp + (t['AfterburnerBoost'] * t['ThrustMult'] -1)* t['Thrust'] * afterburnerThrustKgsToInfraRedBrightness

#band 2(airframe)

def getMissileLocation(time):
    pass

#!===============================================================
# Force CALCULATIONS
#!===============================================================
## ??
wingArea = math.pi *math.pow((m['caliber'])/2,2)*m['wingAreamult']

Thrust = m['force'] * m['timeFire']

#!===============================================================
# other CALCULATIONS
#!===============================================================
def vec_normalize(P1):
    if np.linalg.norm(P1) == 0:
        return P1/99999999
    return P1/np.linalg.norm(P1)
# a = dv/dt
def getAcceleration(P3,P2,P1,dt):
    V3_2 = (P3-P2)/dt
    V2_1 = (P2-P1)/dt
    return np.linalg.norm(V3_2-V2_1)/(dt)
    #return np.linalg.norm(P3-P1)/(dt*dt)
# v = dx/dt
def getSpeed(P2,P1,dt):
    return np.linalg.norm(P2-P1)/dt

def getLinear(lst,height):
    index = min(range(len(lst)),key=lambda i: abs(lst[i][0]-height))
    if index == 0 : index += 1
    aH = lst[index][0];   aV = lst[index][1]
    bH = lst[index-1][0]; bV = lst[index-1][1]
    return bV + (height-bH)*((bV-aV)/(bH-aH))
# height(m)
def getAirDensity(height):
    #https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
    # Air Density table
    airDensityList = [
        (-1000, 1.347), (0, 1.225), 
        (1000, 1.112),  (2000, 1.007), (3000, 0.9093), (4000, 0.8194), (5000, 0.7364),
        (6000, 0.6601), (7000, 0.5900), (8000, 0.5258), (9000, 0.4671), (10000, 0.4135),  
        (15000, 0.1948), (20000, 0.08891), (25000,0.04008),  (30000, 0.01841), (40000,0.003996),
        (50000, 0.001027), (60000,0.0003097), (70000,0.00008283), (80000,0.00001846)
    ]
    return round(getLinear(airDensityList,height),6)

# acceleration(m/s^2) height(m)
def getGravity(height):
    #https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
    # Gravity table
    gravityList = [
        (-1000, 9.810), (0, 9.807), 
        (1000, 9.804), (2000, 9.801), (3000, 9.797), (4000, 9.794), (5000, 9.791),
        (6000, 9.788), (7000, 9.785), (8000, 9.782), (9000, 9.779), (10000, 9.776),  
        (15000, 9.761), (20000, 9.745), (25000, 9.730), (30000, 9.715), (40000,9.684),
        (50000, 9.654), (60000, 9.624), (70000,9.594), (80000, 9.564)
    ]
    return round(getLinear(gravityList,height),4)

def getGForce(acceleration, height):
    return round(acceleration / getGravity(height),1)

# height(m) temperature(C)
def getTemperatureAtHeight(height):
    #https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
    # Atmosphere Temperature table
    temperatureList = [
        (-1000, 21.50), (0, 15.00), 
        (1000, 8.50), (2000, 2.00), (3000,-4.49), (4000, -10.98), (5000, -17.47),
        (6000, -23.96), (7000, -30.45), (8000, -36.94), (9000, -43.42), (10000,-49.90),  
        (15000, -56.50), (20000, -56.50), (25000,-51.60),  (30000, -46.64),  (40000, -22.80),
        (50000, -2.5), (60000,-26.13), (70000,-53.57), (80000, -74.51)
    ]
    return round(getLinear(temperatureList,height),3)
# velocity(m/s), height(m)
def getMach(velocity, height):
    #http://www.aerospaceweb.org/question/atmosphere/q0126.shtml
    # mach = velocity / sound_speed_at_given_altitude
    # sound_speed_at_given_altitude = 331 + 0.6*temperature
    speed_of_sound = 331 + 0.6*getTemperatureAtHeight(height)
    return round(velocity/speed_of_sound,2)


#!===============================================================
fig = plt.figure()
#!===============================================================
# GRAPH 1
#!===============================================================

#!===============================================================
# GRAPH 2
#!===============================================================
trajectoryAx = fig.add_subplot(projection = '3d')

trajectoryAx.set_xlim3d([0,RANGEMAX])
trajectoryAx.set_ylim3d([-RANGEMAX/2,RANGEMAX/2])
trajectoryAx.set_zlim3d([0, 2000])

trajectoryAx.set_xlabel('DISTANCE(m)')
trajectoryAx.set_ylabel('DISTANCE(m)')
trajectoryAx.set_zlabel('Altitude(m)')

def genTargetTrajectory():
    r,rho,theta = INIT_TARGET_HORIZ_COORDINATE
    #horizontal -> cartesian coordinate
    rotated_position = np.array([   r*np.cos(theta),
                                    r*np.cos(rho)*np.sin(theta),
                                    r*np.sin(rho)*np.sin(theta)
                                ])
    # +initial position
    print(rotated_position)
    TARGET_POSITION = INIT_MISSILE_POSITION+rotated_position
    #INIT_TARGET_POSITION = [x+y for x,y in zip(INIT_MISSILE_POSITION,TARGET_POSITION)]
    
    direction_vec = np.array([1.0, 0.0, 0.0])
    newdirection = direction_vec  

    # fly straight, until reaction time
    reaction_time = 0.5 #s
    f = 0
    velocity_per_frame = TARGET_INIT_VELOCITY/FPS
    while f < reaction_time*FPS:
        TARGET_POSITION += np.array([velocity_per_frame,0,0])
        c = []
        for ai in TARGET_POSITION: c.append(ai)
        for bi in newdirection: c.append(bi)
        yield np.array(c)
        #yield np.array(TARGET_POSITION)
        f += 1
    
    def rotate_vec_x(P,angle):
        return np.array([P[0],
                        np.cos(angle)*P[1]-np.sin(angle)*P[2],
                        np.sin(angle)*P[1]+np.cos(angle)*P[2]])
    def rotate_vec_y(P,angle):
        return np.array([np.cos(angle)*P[0]+np.sin(angle)*P[2],
                        P[1],
                        -np.sin(angle)*P[0]+np.cos(angle)*P[2]])
    def rotate_vec_z(P,angle):
        return np.array([np.cos(angle)*P[0]-np.sin(angle)*P[1],
                        np.sin(angle)*P[0]+np.cos(angle)*P[1],
                        P[2]])

    # 2. rotate
    maxrotation = 1.5
    rotation_radius = 1 #m

    downwards_degree = np.radians(30)

    angle = 0
    angle_per_rotate = maxrotation*2*np.pi/(MaxFrame-reaction_time*FPS)
    
    while f < MaxFrame:
        #angular_vector = vec_normalize(np.array([1.0,np.cos(angle),np.sin(angle)]))
        newdirection = direction_vec + np.array([0,rotation_radius,0])
        newdirection = rotate_vec_x(newdirection,angle)
        newdirection = rotate_vec_y(newdirection,downwards_degree)
        #newdirection = vec_normalize(direction_vec+rotation_radius*angular_vector)
        newdirection = vec_normalize(newdirection)

        accel_gravity = getGravity(TARGET_POSITION[2])
        velocity_per_frame += accel_gravity/(FPS*FPS)
        if velocity_per_frame > t['maxspeed']/FPS:
            velocity_per_frame = t['maxspeed']/FPS
        
        TARGET_POSITION += velocity_per_frame*newdirection
        #print(np.concatenate((TARGET_POSITION,newdirection)))
        c = []
        for ai in TARGET_POSITION: c.append(ai)
        for bi in newdirection: c.append(bi)
        yield np.array(c)

        angle += angle_per_rotate
        f += 1
data_target_raw = list(genTargetTrajectory())
data_raw_position = list()
data_raw_direction = list()
for elm in data_target_raw:
    data_raw_position.append(elm[:3])
    data_raw_direction.append(elm[3:])
data_target_position = np.array(data_raw_position).T
data_target_direction = np.array(data_raw_direction).T
name_target = t['name']
line_target,= trajectoryAx.plot(0,0,0)
text_target = trajectoryAx.text(0,0,0,'', color='blue')
line_x_target,= trajectoryAx.plot(0,0,0)
line_y_target,= trajectoryAx.plot(0,0,0)
line_z_target,= trajectoryAx.plot(0,0,0)

def genMissileTrajectory():
    #INIT_MISSILE_POSITION
    """
    f = 0
    while f < MaxFrame:
        yield getMissileLocation(f/FPS)
        f += 1
    """
    phi = 0
    while phi < 2*np.pi:
        yield np.array([np.cos(phi), np.sin(phi), phi, True])
        phi += 2*np.pi/MaxFrame
    #"""

# ax.plot returns list of line2d
data_missile = np.array(list(genMissileTrajectory())).T
name_missile = m['name']
line_missile,= trajectoryAx.plot(0,0,0)
text_missile = trajectoryAx.text(0,0,0,'', color='red')


USE_FLARE = False; show_flare = False
show_flare_at = 4
text_flare = trajectoryAx.text(0,0,0,'FLARE', color='purple')
text_flare.set_visible(False)

show_boom = False
text_boom = trajectoryAx.text(0,0,0,'Boom', color='orange')
text_boom.set_visible(False)

#! update function
def update(time):
    global USE_FLARE; global show_flare
    # missile info update
    mdata = data_missile
    line_missile.set_data(mdata[:2,:time])
    line_missile.set_3d_properties(mdata[2,:time])
    text_missile.set_position(mdata[:3,time])

    mP1 = mdata[:3,time]
    if time == 0: mP3 = mP2 = mP1
    elif time == 1: mP3 = mP2 = mdata[:3,time-1]
    else: mP2 = mdata[:3,time-1]; mP3 = mdata[:3,time-2]
    mheight = mdata[2,time] # z
    mSpeed = getSpeed(mP1,mP2,1/FPS)
    text_missile.set_text(name_missile+'\n'+
                            #'Mach: '+ str(getMach(mSpeed,mheight))+'('+str(round(mSpeed/0.277,2))+'km/h)\n'+
                            'Mach: '+ str(getMach(mSpeed,mheight))+'\n'+
                            'AoA: ' +'\n'+
                            'G: ' + str(getGForce(getAcceleration(mP1,mP2,mP3,1/FPS),mheight))
                        )
    if not mdata[3,time]: text_missile.set_color('grey')

    # target info update
    tdata = data_target_position
    line_target.set_data(tdata[:2,:time])
    line_target.set_3d_properties(tdata[2,:time])
    
    tP1 = tdata[:3,time]
    if time == 0: tP3 = tP2 = tP1
    elif time == 1: tP3 = tP2 = tdata[:3,time-1]
    else: tP2 = tdata[:3,time-1]; tP3 = tdata[:3,time-2]
    theight = tdata[2,time] # z
    tSpeed = getSpeed(tP1,tP2,1/FPS)

    text_target.set_position(tP1)
    text_target.set_text(name_target+'\n'+
                            'Mach: '+ str(getMach(tSpeed,theight))+'('+str(round(tSpeed/0.277,2))+'km/h)\n'+
                            'Height: ' + str(round(theight,1))+'m\n'+
                            'G: ' + str(getGForce(getAcceleration(tP1,tP2,tP3,1/FPS),theight))+'\n'+
                            'M-Dist: ' + str(round(np.abs(np.linalg.norm(tP1-mP1))/1000,2))+'km'
                        )
    
    # direction!
    tddata = data_target_direction
    pole_length=300
    # x(acceleration) == data(3~5)
    target_acceleration = tddata[:3,time]
    x_pole_vec = target_acceleration
    x_pole = np.array([tP1,tP1+pole_length*x_pole_vec]).T
    line_x_target.set_data(x_pole[:2])
    line_x_target.set_3d_properties(x_pole[2])
    # y = x cross prev
    #prev_acceleration = tddata[:3,time-1]
    #y_pole_vec = vec_normalize(np.cross(target_acceleration,prev_acceleration))
    y_pole_vec = vec_normalize(np.cross(tP2,tP1))
    y_pole = np.array([tP1,tP1+pole_length*y_pole_vec]).T
    line_y_target.set_data(y_pole[:2])
    line_y_target.set_3d_properties(y_pole[2])
    # z = x cross y
    z_pole_vec = vec_normalize(np.cross(target_acceleration,y_pole_vec))
    z_pole = np.array([tP1,tP1+pole_length*z_pole_vec]).T
    line_z_target.set_data(z_pole[:2])
    line_z_target.set_3d_properties(z_pole[2])
    

    #Flare!
    if USE_FLARE:
        if time == 0:
            text_flare.set_visible(False); show_flare = False
        elif not show_flare and show_flare_at < (TIMEMAX*time/MaxFrame):
            text_flare.set_position(tdata[:3,time])
            text_flare.set_visible(True); show_flare = True
    

animation = ani.FuncAnimation(
    fig=fig,
    func=update,
    frames=MaxFrame,
    interval = TIMEMAX * 1000/MaxFrame
)
#animation.save('test.webp',writer='imagemagick')
plt.show()
