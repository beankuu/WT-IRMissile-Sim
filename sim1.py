## up left 3D graph, simulating entire view
import mObjects as mobj
from mObjects import Vec3D as vec3
import data
import numpy as np

def init(plot,datapack):
    """
    initializing plot,
    generating 3d data to plot

    Args:
        plot (matplotlib.pyplot.subplot) : matplotlib plot to be drawn
        datapack ([targetData, missileData, flaresData]) : [list of target, list of missile, list of flare] data

    Returns:
        list: list of [list of [matplotlib plot,matplotlib text...]...]
    """
    targetData,missileData,flaresData = datapack

    plot.set_xlim3d([0,7000])
    plot.set_ylim3d([-1000,1000])
    plot.set_zlim3d([2500,3500])

    plot.set_xlabel('거리(m)')
    plot.set_ylabel('거리(m)')
    plot.set_zlabel('고도(m)')

    plot.tick_params(axis='x',labelsize='x-small')
    plot.tick_params(axis='y',labelsize='x-small')
    plot.tick_params(axis='z',labelsize='x-small')

    #==================================================================
    text_frame_now = plot.text3D(plot.get_xlim3d()[0],
                                    plot.get_ylim3d()[0],
                                    plot.get_zlim3d()[0],'', color='black')
    #-----------------------------------------------------------
    # list of objects to Plot
    ## PlotObjects related to Target Object
    color = 'blue'
    targetObjects = [plot.plot(0,0,0, color=color,linewidth=0.5),
                        plot.text3D(0,0,0,'', color=color, size='small'),
                        targetData]

    color = 'red'
    missileObjects = [plot.plot(0,0,0, color=color,linewidth=0.5), #mainplot
                        plot.text3D(0,0,0,'', color=color, ha='right', size='small'),
                        missileData,
                        plot.plot(0,0,0, color='magenta',linewidth=1), #fvec
                        plot.plot(0,0,0, color='cyan',linewidth=1), #upvec
                        plot.plot(0,0,0, color='green',linewidth=0.2) #sVec
                    ]

    color = 'purple'
    flareObjects = [
                    [
                        plot.text3D(0,0,0,'*', color=color, size='x-large'),
                        flaresData[i]
                    ] for i in range(len(flaresData))
                    ]
    return [targetObjects,missileObjects,flareObjects,[text_frame_now]]
#-----------------------------------------------------------
def resetColor(resetObject,objType):
    """
    resets color of mathplotlib plot for given object Type

    sets color of matplotlib plot, for given object type
    Args:
        resetObject (matplotlib plot) : given plot
        objType (string) : object type in string
    """
    if objType == 'target': color = 'blue'
    elif objType == 'missile': color = 'red'
    elif objType == 'flare': color = 'purple'
    else: color = 'grey'
    resetObject.set_color(color)

def genTextStr(obj):
    """
    generates string to be used by matplotlib text

    Args:
        obj (mObjects.SimObject) : SimObject (Flare/Missile/Target)
    Returns:
        string : text
    """
    text = obj.data['name']
    if type(obj) == mobj.FlareObject:
        return text
    
    mach = round(data.calcMach(vec3.norm(obj.vVec),obj.pVec.z),1)
    speed = round(vec3.norm(obj.vVec)/0.277,1)
    text += '\n'+'Mach: ' + str(mach)+'('+str(speed)+'km/h)\n'+\
            'height: ' + str(round(obj.pVec.z,1))+'m'
    if type(obj) == mobj.MissileObject:
        return text

    text += '\nAF' if obj.isAfterburnerOn else ''
    #if type(obj) == mobj.TargetObject:
    return text

def update(frame,simPlots):
    """
    update of plot, per frame

    Args:
        frame (float) : frame(time)
        simPlots (list of object data) : packed objects to be updated
    """
    targetObjects,missileObjects,flareObjects,otherObjects = simPlots
    otherObjects[0].set_text(str(frame)+'('+str(round(frame*data.dt,1))+'s)')
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
    line_object, text_object, missileObjectList, line_fvec, line_upvec, line_svec = missileObjects
    # basic trajectories
    direction_vectors = [o.pVec for o in missileObjectList[:frame+1]]
    line_object[0].set_data_3d([o.x for o in direction_vectors],
                            [o.y for o in direction_vectors],
                            [o.z for o in direction_vectors])
    
    missile = missileObjectList[frame]
    barlen = 300
    bardata = missile.pVec+barlen*missile.fVec
    line_fvec[0].set_data_3d([missile.pVec.x, bardata.x],
                             [missile.pVec.y, bardata.y],
                             [missile.pVec.z, bardata.z])
    bardata = missile.pVec+barlen*missile.upVec
    line_upvec[0].set_data_3d([missile.pVec.x, bardata.x],
                             [missile.pVec.y, bardata.y],
                             [missile.pVec.z, bardata.z])
    bardata = missile.pVec+barlen*missile.sVec
    line_svec[0].set_data_3d([missile.pVec.x, bardata.x],
                             [missile.pVec.y, bardata.y],
                             [missile.pVec.z, bardata.z])

    #set_position only fetches first 2(x,y)
    text_object.set_position_3d(missile.pVec.toList())
    text_object.set_text(genTextStr(missile))

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
    if targetObjectList[frame].isHit:
        targetObjects[1].set_color('grey');
    if missileObjectList[frame].isHit: 
        missileObjects[1].set_color('grey')
    
    #------------------------------------------
    ## rotate graph
    #ax.view_init(elev = 20-10+frame/5, azim=-160+30-frame/3)

    ## do-things at end of frame
    if frame == data.MaxFrame-1:
        resetColor(targetObjects[1],'target')
        resetColor(missileObjects[1],'missile')
        for flareObj in flareObjects:
            resetColor(flareObj[0],'flare')