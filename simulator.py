import numpy as np

from matplotlib import rc
rc('font', family='Gulim')

import matplotlib.pyplot as plt
import matplotlib.animation as ani

# customs
import pathGen
import data
import mObjects as mobj
from mObjects import Vec3D as vec3

#===============================================
#-----------------------------------------------

target =  mobj.TargetObject(data=data.target1,    pVec = vec3(3000,0,3000),  vVec=vec3(280, 0, 0) )
missile = mobj.MissileObject(data=data.missile2,  pVec = vec3(0,0,3000),     sVec=vec3(3000,0,3000) )

flareTypeData = data.flare0 if target.data['flareType'] == 0 else data.flare1
flareTimes = [0.0,7.0]
flares = [ mobj.FlareObject(data=flareTypeData) for i in range(len(flareTimes)) ]

## list of [list of [SimObjects per frame]]...
targetData, missileData, flaresData = pathGen.genPaths(target,missile,[flares,flareTimes])

#==================================================================

fig = plt.figure()
trajectoryPlot = fig.add_subplot(projection = '3d')

trajectoryPlot.set_xlim3d([0,7000])
trajectoryPlot.set_ylim3d([-100,1500])
trajectoryPlot.set_zlim3d([2000,3000])

trajectoryPlot.set_xlabel('거리(m)')
trajectoryPlot.set_ylabel('거리(m)')
trajectoryPlot.set_zlabel('고도(m)')

#==================================================================
text_frame_now = trajectoryPlot.text3D(0,0,1500,'', color='black')
#-----------------------------------------------------------
# list of objects to Plot
## PlotObjects related to Target Object
color = 'blue'
targetObjects = [trajectoryPlot.plot(0,0,0, color=color),
                    trajectoryPlot.text3D(0,0,0,'', color=color, size='x-small'),
                    targetData]

color = 'red'
missileObjects = [trajectoryPlot.plot(0,0,0, color=color),
                    trajectoryPlot.text3D(0,0,0,'', color=color, size='x-small'),
                    missileData]

color = 'purple'
flareObjects = [
                [
                    trajectoryPlot.text3D(0,0,0,'*', color=color, size='x-large'),
                    flaresData[i]
                ] for i in range(len(flareTimes))
                ]
#-----------------------------------------------------------
def isHit(t, m):
    mtDistance = (m.pVec-t.pVec).norm()
    ## Method 1: minimum distance between two vectors
    #vVecCross = mP.upVec if mP.vVec.normalize() == tP.vVec.normalize() else mP.vVec.normalize().cross(tP.vVec.normalize())
    #maxDx = (mP.pVec-tP.pVec).dot(vVecCross)/(vVecCross.norm()*vVecCross.norm())
    ## Method 2: mindist Point + v*dt as radius
    maxDx = (m.vVec.norm() if t.vVec.norm() < m.vVec.norm() else t.vVec.norm()) * data.dt
    # missile? or other types?
    proximityDist = m.data['proximityFuse_radius']

    return mtDistance < proximityDist+maxDx

def resetColor(resetObject,objType):
    if objType == 'target': color = 'blue'
    elif objType == 'missile': color = 'red'
    elif objType == 'flare': color = 'purple'
    else: color = 'grey'
    resetObject.set_color(color)

def genTextStr(obj):
    if type(obj) == mobj.TargetObject or type(obj) == mobj.MissileObject:
        mach = round(data.calcMach(obj.vVec.norm(),obj.pVec.z),1)
        speed = round(obj.vVec.norm()/0.277,1)
        return  obj.data['name']+'\n'+\
            'Speed: ' + str(speed)+'km/h'+\
            'Mach: ' + str(mach)+'\n'
    elif type(obj) == mobj.FlareObject: 
        return  obj.data['name']
    else:
        return ''

#! update function
def update(frame):
    text_frame_now.set_text(str(frame)+'('+str(round(frame*data.dt,1))+'s)')
    #------------------------------------------
    ## target
    line_object, text_object, targetObjectList = targetObjects
    # basic trajectories
    direction_vectors = [o.pVec for o in targetObjectList[:frame+1]]
    line_object[0].set_data_3d([o.x for o in direction_vectors],
                            [o.y for o in direction_vectors],
                            [o.z for o in direction_vectors])
    #set_position only fetches first 2(x,y)
    text_object.set_position_3d(targetObjectList[frame].pVec.toList())
    text_object.set_text(genTextStr(targetObjectList[frame]))

    #------------------------------------------
    ## missile
    line_object, text_object, missileObjectList = missileObjects
    # basic trajectories
    direction_vectors = [o.pVec for o in missileObjectList[:frame+1]]
    line_object[0].set_data_3d([o.x for o in direction_vectors],
                            [o.y for o in direction_vectors],
                            [o.z for o in direction_vectors])
    #set_position only fetches first 2(x,y)
    text_object.set_position_3d(missileObjectList[frame].pVec.toList())
    text_object.set_text(genTextStr(missileObjectList[frame]))

    #------------------------------------------
    ## flare s 
    for text_object, flareObjectList in flareObjects:
        text_object.set_position_3d(flareObjectList[frame].pVec.toList())

    #------------------------------------------
    ## if condition
    if missileObjectList[frame].isLocked:
        missileObjects[1].set_color('red')
    else:
        missileObjects[1].set_color('grey')
    if isHit(targetObjectList[frame],missileObjectList[frame]): 
        targetObjects[1].set_color('grey'); missileObjects[1].set_color('grey')
    
    #------------------------------------------
    ## rotate graph
    trajectoryPlot.view_init(elev = 20-10+frame/8, azim=-160+30-frame/3)

    ## do-things at end of frame
    if frame == data.MaxFrame-1:
        resetColor(targetObjects[1],'target')
        resetColor(missileObjects[1],'missile')
        for flareObj in flareObjects:
            resetColor(flareObj[0],'flare')
#---------------------------------------------------------------------------
animation = ani.FuncAnimation(
    fig = fig,
    func = update,
    fargs = (),
    frames = data.MaxFrame,
    interval = 1000/data.FPS
)
#FFwriter=ani.FFMpegWriter(fps=15, extra_args=['-vcodec', 'libx264'])
FFwriter=ani.FFMpegWriter(fps=15)
#animation.save('test.mp4',writer = FFwriter)
plt.show()
