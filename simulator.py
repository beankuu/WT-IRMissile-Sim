import numpy as np

import matplotlib.pyplot as plt
import matplotlib.animation as ani
from mpl_toolkits.mplot3d import Axes3D

# customs
import pathGen
import data
import calculate as calc
import mObjects as obj

fig = plt.figure()
trajectoryAx = fig.add_subplot(projection = '3d')

trajectoryAx.set_xlim3d([0,data.RANGEMAX])
trajectoryAx.set_ylim3d([-data.RANGEMAX/2,data.RANGEMAX/2])
trajectoryAx.set_zlim3d([0, 2000])

trajectoryAx.set_xlabel('DISTANCE(m)')
trajectoryAx.set_ylabel('DISTANCE(m)')
trajectoryAx.set_zlabel('Altitude(m)')

data_target_raw = list(pathGen.genTargetTrajectory())
data_raw_position = list()
data_raw_direction = list()
for elm in data_target_raw:
    data_raw_position.append(elm[:3])
    data_raw_direction.append(elm[3:])
data_target_position = np.array(data_raw_position).T
data_target_direction = np.array(data_raw_direction).T
name_target = data.t['name']
line_target,= trajectoryAx.plot(0,0,0)
text_target = trajectoryAx.text(0,0,0,'', color='blue')
line_x_target,= trajectoryAx.plot(0,0,0)
line_y_target,= trajectoryAx.plot(0,0,0)
line_z_target,= trajectoryAx.plot(0,0,0)

# ax.plot returns list of line2d
data_missile = np.array(list(pathGen.genMissileTrajectory())).T
name_missile = data.m['name']
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
def update(time,USE_FLARE,show_flare):
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
    mSpeed = calc.calcSpeed(mP1,mP2,1/data.FPS)
    text_missile.set_text(name_missile+'\n'+
                            #'Mach: '+ str(getMach(mSpeed,mheight))+'('+str(round(mSpeed/0.277,2))+'km/h)\n'+
                            'Mach: '+ str(data.getMach(mSpeed,mheight))+'\n'+
                            'AoA: ' +'\n'+
                            'G: ' + str(data.getGForce(calc.calcAcceleration(mP1,mP2,mP3,1/data.FPS),mheight))
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
    tSpeed = calc.calcSpeed(tP1,tP2,1/data.FPS)

    text_target.set_position(tP1)
    text_target.set_text(name_target+'\n'+
                            'Mach: '+ str(data.getMach(tSpeed,theight))+'('+str(round(tSpeed/0.277,2))+'km/h)\n'+
                            'Height: ' + str(round(theight,1))+'m\n'+
                            'G: ' + str(data.getGForce(calc.calcAcceleration(tP1,tP2,tP3,1/data.FPS),theight))+'\n'+
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
    y_pole_vec = calc.vec_normalize(np.cross(tP2,tP1))
    y_pole = np.array([tP1,tP1+pole_length*y_pole_vec]).T
    line_y_target.set_data(y_pole[:2])
    line_y_target.set_3d_properties(y_pole[2])
    # z = x cross y
    z_pole_vec = calc.vec_normalize(np.cross(target_acceleration,y_pole_vec))
    z_pole = np.array([tP1,tP1+pole_length*z_pole_vec]).T
    line_z_target.set_data(z_pole[:2])
    line_z_target.set_3d_properties(z_pole[2])
    

    #Flare!
    if USE_FLARE:
        if time == 0:
            text_flare.set_visible(False); show_flare = False
        elif not show_flare and show_flare_at < (data.TIMEMAX*time/data.MaxFrame):
            text_flare.set_position(tdata[:3,time])
            text_flare.set_visible(True); show_flare = True
    

animation = ani.FuncAnimation(
    fig = fig,
    func = update,
    fargs = (USE_FLARE,show_flare),
    frames = data.MaxFrame,
    interval = data.TIMEMAX * 1000/data.MaxFrame
)
#animation.save('test.webp',writer='Pillow')
plt.show()
