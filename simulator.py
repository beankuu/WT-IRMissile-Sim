import numpy as np

from matplotlib import rc
rc('font', family='Gulim')

import matplotlib.pyplot as plt
import matplotlib.animation as ani
from mpl_toolkits.mplot3d import proj3d

# customs
import pathGen
import data
import calculate as calc
import mObjects as obj
from mObjects import Vec3D as vec3

#===============================================
#-----------------------------------------------
targetdata = data.target1
missiledata = data.missile1
#-----------------------------------------------

fig = plt.figure()
trajectoryPlot = fig.add_subplot(projection = '3d')


trajectoryPlot.set_xlim3d([0,7000])
trajectoryPlot.set_ylim3d([-2000,100])
trajectoryPlot.set_zlim3d([1500,3500])

trajectoryPlot.set_xlabel('거리(m)')
trajectoryPlot.set_ylabel('거리(m)')
trajectoryPlot.set_zlabel('고도(m)')

#==================================================================
text_frame_now = trajectoryPlot.text3D(0,0,0,'', color='black')
#-----------------------------------------------------------
# list of mObjects.TargetObject
data_target = list(pathGen.genTargetTrajectory(obj.TargetObject(data=targetdata)))
name_target = data_target[0].data['name']
line_target,= trajectoryPlot.plot(0,0,0, color='blue')
text_name_target = trajectoryPlot.text3D(0,0,0,'', color='blue', va='top', ha='left')
text_tags_target = trajectoryPlot.text3D(0,0,0,'', color='blue', va='top', ha='left', size='x-small')

line_x_target,= trajectoryPlot.plot(0,0,0, color = 'magenta')
line_y_target,= trajectoryPlot.plot(0,0,0, color = 'lime')
line_z_target,= trajectoryPlot.plot(0,0,0, color = 'cyan')
#-----------------------------------------------------------
# list of mObjects.MissileObject
data_missile = list(pathGen.genMissileTrajectory(obj.MissileObject(data=missiledata), data_target))
name_missile = data_missile[0].data['name']
line_missile,= trajectoryPlot.plot(0,0,0, color = 'red')
text_name_missile = trajectoryPlot.text3D(0,0,0,'', color='red', va='top', ha='right')
text_tags_missile = trajectoryPlot.text3D(0,0,0,'', color='red', va='top', ha='right', size='x-small')

line_seeker_missile,= trajectoryPlot.plot(0,0,0, color = 'yellow')
#-----------------------------------------------------------
# Flares!
USE_FLARE = False
show_flare_at = 4
text_flare = trajectoryPlot.text3D(0,0,0,'FLARE', color='purple')
text_flare.set_visible(False)

# Boom when hit
"""
show_boom = False
text_boom = trajectoryAx.text3D(0,0,0,'Boom', color='orange')
text_boom.set_visible(False)
"""

mindist = [data.INFINITE, 0] #[any big number, max location diff at frame]
#! update function
def update(frame,mindist):
    text_frame_now.set_text(str(frame)+'('+str(round(frame*data.dt,1))+'s)')
    #------------------------------------------
    # basic trajectories
    direction_vectors = [o.pVec for o in data_missile[:frame+1]]
    line_missile.set_data_3d([o.x for o in direction_vectors],
                             [o.y for o in direction_vectors],
                             [o.z for o in direction_vectors])
    direction_vectors = [o.pVec for o in data_target[:frame+1]]
    line_target.set_data_3d([o.x for o in direction_vectors],
                            [o.y for o in direction_vectors],
                            [o.z for o in direction_vectors])
    #------------------------------------------
    #points for updating infos
    mP = data_missile[frame]
    tP = data_target[frame]
    #------------------------------------------
    # missile info update
    mHeight = mP.pVec.z
    mSpeed = mP.vVec.norm()/data.dt

    #set_position only fetches first 2(x,y)
    text_name_missile.set_position_3d(mP.pVec.toList())
    text_tags_missile.set_position_3d(mP.pVec.toList())
    text_name_missile.set_text(name_missile)
    text_tags_missile.set_text('\n\n'+
                            'Mach: '+ str(round(data.calcMach(mSpeed,mHeight),1))+'('+str(round(mSpeed/0.277,1))+'km/h)\n'+
                            #'Mach: '+ str(data.getMach(mSpeed,mHeight))+'\n'+
                            'AoA: ' +'몰?루\n'+
                            'G: ' + str(round(mP.aVec.norm()/data.dt/data.getGravity(mHeight),1))+'\n'+
                            'to-Target: ' + str(round(np.abs((tP.pVec-mP.pVec).norm())/1000,1))+'km'
                        )
    if mP.isLocked: text_name_missile.set_color('red'); text_tags_missile.set_color('red')
    else: text_name_missile.set_color('grey');text_tags_missile.set_color('grey')

    # direction!
    pole_length=300
    # x(acceleration) == data(3~5)
    seeker_pole_vec = mP.sVec
    seeker_pole_end = mP.pVec+pole_length*seeker_pole_vec
    seeker_pole = ([[mP.pVec.x,seeker_pole_end.x], 
                    [mP.pVec.y,seeker_pole_end.y],
                    [mP.pVec.z,seeker_pole_end.z]])
    line_seeker_missile.set_data_3d(seeker_pole)
    #------------------------------------------
    # target info update 
    
    tHeight = tP.pVec.z
    tSpeed = tP.vVec.norm()/data.dt

    text_name_target.set_position_3d(tP.pVec.toList())
    text_tags_target.set_position_3d(tP.pVec.toList())
    text_name_target.set_text(name_target)
    text_tags_target.set_text('\n\n'+
                            'Speed: ' + str(round(tSpeed/0.277,1)) +'km/h\n'+
                            'Mach: ' + str(round(data.calcMach(tSpeed,tHeight),1))+'\n'+
                            'Height: ' + str(round(tHeight,1)) +'m\n'+
                            'G: ' + str(round(tP.aVec.norm()/data.dt/data.getGravity(tHeight),1))
                        )
    if frame == 0: text_name_target.set_color('blue');text_tags_target.set_color('blue')

    mtDistance = (mP.pVec-tP.pVec).norm()
    maxDx = (mP.vVec.norm() if mP.vVec.norm() > tP.vVec.norm() else tP.vVec.norm())#*data.dt
    if mtDistance < mP.data['proximityFuse_radius']+maxDx: text_name_target.set_color('grey');text_tags_target.set_color('grey')
    if mtDistance < mindist[0]: mindist[0],mindist[1] = mtDistance, maxDx

    # direction!
    pole_length=150
    # x = direction(velocity)
    x_pole_vec = tP.vVec.normalize()
    x_pole_end = tP.pVec+2*pole_length*x_pole_vec #? 2x?
    x_pole = ([[tP.pVec.x, x_pole_end.x], 
               [tP.pVec.y, x_pole_end.y],
               [tP.pVec.z, x_pole_end.z]])
    line_x_target.set_data_3d(x_pole)
    # z = upwards(up)
    z_pole_vec = tP.upVec # assume normalized
    z_pole_end = tP.pVec+pole_length*z_pole_vec
    z_pole = ([[tP.pVec.x, z_pole_end.x], 
               [tP.pVec.y, z_pole_end.y],
               [tP.pVec.z, z_pole_end.z]])
    line_z_target.set_data_3d(z_pole)
    # y = (x) cross (up)
    y_pole_vec = x_pole_vec.cross(z_pole_vec)
    y_pole_end = tP.pVec+pole_length*y_pole_vec
    y_pole = ([[tP.pVec.x, y_pole_end.x], 
               [tP.pVec.y, y_pole_end.y],
               [tP.pVec.z, y_pole_end.z]])
    line_y_target.set_data_3d(y_pole)
    
    #------------------------------------------
    #Flare!
    if USE_FLARE:
        if frame == 0:
            text_flare.set_visible(False)
        elif not text_flare.get_visible() and show_flare_at < frame*data.dt:
            text_flare.set_position_3d(tP.toList())
            text_flare.set_visible(True)


    # rotate graph
    trajectoryPlot.view_init(elev = 20, azim=-160+30-frame/3)
    # at end of interval
    if frame == data.MaxFrame-1:
        print(mindist); mindist = [data.INFINITE,0]
#---------------------------------------------------------------------------
animation = ani.FuncAnimation(
    fig = fig,
    func = update,
    fargs = (mindist,),
    frames = data.MaxFrame,
    interval = 1000/data.FPS
)
#animation.save('test.webp')
plt.show()
