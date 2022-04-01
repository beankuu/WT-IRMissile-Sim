## up left 3D graph, simulating entire view
import mObjects as mobj
import data
import numpy as np

def init(plot,datapack):
    targetData,missileData,flaresData = datapack

    plot.set_xlim3d([0,7000])
    plot.set_ylim3d([-50,200])
    plot.set_zlim3d([2700,3000])

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
    missileObjects = [plot.plot(0,0,0, color=color,linewidth=0.5),
                        plot.text3D(0,0,0,'', color=color, ha='right', size='small'),
                        missileData]

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
    if objType == 'target': color = 'blue'
    elif objType == 'missile': color = 'red'
    elif objType == 'flare': color = 'purple'
    else: color = 'grey'
    resetObject.set_color(color)

def genTextStr(obj):
    if type(obj) == mobj.TargetObject:
        mach = round(data.calcMach(obj.vVec.norm(),obj.pVec.z),1)
        speed = round(obj.vVec.norm()/0.277,1)
        afText = '\nAF' if obj.isAfterburnerOn else ''
        return  obj.data['name']+'\n'+\
            'Mach: ' + str(mach)+'('+str(speed)+'km/h)\n'+\
            'height: ' + str(round(obj.pVec.z,1))+'m'+\
            afText
    elif type(obj) == mobj.MissileObject:
        mach = round(data.calcMach(obj.vVec.norm(),obj.pVec.z),1)
        speed = round(obj.vVec.norm()/0.277,1)
        return  obj.data['name']+'\n'+\
            'Mach: ' + str(mach)+'('+str(speed)+'km/h)\n'+\
            'height: ' + str(round(obj.pVec.z,1))+'m'
    elif type(obj) == mobj.FlareObject: 
        return  obj.data['name']
    else:
        return ''

def update(frame,ax,simPlots):
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