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
    targetData,missilesData,flaresData = datapack

    #plot.set_xlim3d([0,7000])
    #plot.set_ylim3d([-1000,1000])
    #plot.set_zlim3d([2500,3500])

    plot.set_xlim3d([2000,5000])
    plot.set_ylim3d([-250,250])
    plot.set_zlim3d([2900,3100])
    

    #plot.set_xlim3d([0,7000])
    #plot.set_ylim3d([-3000,3000])
    #plot.set_zlim3d([500,7500])

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
                        plot.text3D(0,0,0,'', color=color, va='top', ha='right', size='x-small'),
                        targetData]

    color = 'red'
    missilesObjects = [
                        [
                            plot.plot(0,0,0, color=color,linewidth=0.5), #mainplot
                            plot.text3D(0,0,0,'', color=color, ha='right', size='small'),
                            missilesData[i],
                            plot.plot(0,0,0, color='magenta',linewidth=1), #fvec
                            plot.plot(0,0,0, color='cyan',linewidth=1), #upvec
                            plot.plot(0,0,0, color='green',linewidth=0.5) #sVec
                        ] for i in range(len(missilesData))
                    ]

    color = 'purple'
    flareObjects = [
                    [
                        plot.text3D(0,0,0,'*', color=color, size='x-large'),
                        flaresData[i]
                    ] for i in range(len(flaresData))
                    ]
    return [targetObjects,missilesObjects,flareObjects,[text_frame_now]]
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
    speed = vec3.norm(obj.vVec)
    speedKmh = round(speed/0.277,1)
    text += '\n'+'Mach: ' + str(mach)+'('+str(speedKmh)+'km/h)\n'

    if type(obj) == mobj.MissileObject:
        energy = round(0.5*speed*speed*obj.data['massEnd']/1000,1)
        #text += str(energy)+' KJ'
        return text

    text += '\nAF\n\n' if obj.isAfterburnerOn else '\n\n\n'
    #if type(obj) == mobj.TargetObject:
    return text

def update(frame,simPlots,ax):
    """
    update of plot, per frame

    Args:
        frame (float) : frame(time)
        simPlots (list of object data) : packed objects to be updated
    """
    targetObjects,missilesObjects,flareObjects,otherObjects = simPlots
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
    for line_object, text_object, missileObjectList, line_1, line_2, line_3 in missilesObjects:
    #for line_object, text_object, missileObjectList in missilesObjects:
        text_object.set_position_3d(missileObjectList[frame].pVec.toList())
        text_object.set_text(genTextStr(missileObjectList[frame]))

        # basic trajectories
        direction_vectors = [o.pVec for o in missileObjectList[:frame+1]]
        line_object[0].set_data_3d([o.x for o in direction_vectors],
                                [o.y for o in direction_vectors],
                                [o.z for o in direction_vectors])
        
        missile = missileObjectList[frame]

        barlen = 50
        fVec = missile.fVec
        #rVec = vec3(0,1,0) if fVec == vec3(0,0,-1) else vec3.cross(fVec,vec3(0,0,-1))
        #upVec = vec3.cross(fVec,rVec)
        #upVec = missile.upVec
        bardata = missile.pVec+barlen*fVec
        line_1[0].set_data_3d([missile.pVec.x, bardata.x],
                                [missile.pVec.y, bardata.y],
                                [missile.pVec.z, bardata.z])

        bardata = missile.pVec+barlen*vec3.normalize(missile.vVec)
        line_2[0].set_data_3d([missile.pVec.x, bardata.x],
                                [missile.pVec.y, bardata.y],
                                [missile.pVec.z, bardata.z])
        bardata = missile.pVec+barlen*missile.sVec
        line_3[0].set_data_3d([missile.pVec.x, bardata.x],
                                [missile.pVec.y, bardata.y],
                                [missile.pVec.z, bardata.z])
        
    #------------------------------------------
    ## flare s 
    for text_object, flareObjectList in flareObjects:
        text_object.set_position_3d(flareObjectList[frame].pVec.toList())

    #------------------------------------------
    ## if condition
    for line_object, text_object, missileObjectList, line_1, line_2, line_3 in missilesObjects:
    #for line_object, text_object, missileObjectList in missilesObjects:
        if missileObjectList[frame].isLocked:
            text_object.set_color('red')
        else:
            text_object.set_color('#663333')
        if missileObjectList[frame].isHit: 
            text_object.set_color('#AAAAAA')

    if targetObjectList[frame].isHit:
        targetObjects[1].set_color('#AAAAAA');
        
    
    #------------------------------------------
    ## rotate graph
    #ax.view_init(elev = 10+20-10+frame/5, azim=160-30-frame/3)
    ax.view_init(elev = 10+20-10+frame/5, azim=-30-frame/3)
    """
    if frame < data.MaxFrame*0.1:
        elev = ax.elev+0.4; azim = ax.azim+0.1
    elif frame < data.MaxFrame*0.3:
        elev = ax.elev+0.35; azim = ax.azim+0.15
    elif frame < data.MaxFrame*0.5:
        elev = ax.elev+0.3; azim = ax.azim+0.2
    elif frame < data.MaxFrame*0.7:
        elev = ax.elev+0.1; azim = ax.azim+0.3
    else:
        elev = ax.elev+0.1; azim = ax.azim+0.4
    ax.view_init(elev=elev,azim=azim)
    """

    ## do-things at end of frame
    if frame == data.MaxFrame-1:
        resetColor(targetObjects[1],'target')
        for line_object, text_object, missileObjectList, line_1, line_2, line_3 in missilesObjects:
        #for line_object, text_object, missileObjectList in missilesObjects:
            resetColor(text_object,'missile')
        for text_object, flareObjectList in flareObjects:
            resetColor(text_object,'flare')