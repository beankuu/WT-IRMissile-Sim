## down left 2D graph, Missile forward View
import mObjects as mobj
from mObjects import Vec3D as vec3
import data
import numpy as np

def init(plot,datapack):
    """
    initializing plot,
    generating 2d data to plot

    Args:
        plot (matplotlib.pyplot.subplot) : matplotlib plot to be drawn
        datapack ([targetData, missileData, flaresData]) : [list of target, list of missile, list of flare] data

    Returns:
        list: list of [list of [matplotlib plot,matplotlib text...]...]
    """
    targetData,missileData,flaresData = datapack
    maxAngle = missileData[0].data['g_angleMax']
    maxRange = missileData[0].data['rangeBand0']
    maxFOV = missileData[0].data['g_fov']/2
    plot.set_xlim([-maxAngle*1.2,maxAngle*1.2])
    plot.set_ylim([-maxAngle*1.2,maxAngle*1.2])
    plot.set_xlabel('fov')
    plot.set_ylabel('fov')
    # to right side
    plot.yaxis.tick_right()
    plot.yaxis.set_label_position("right")
    #=================================================================
    # MaxAngle circle
    maxLockcircleData = []
    initLockCircleData = []
    maxLockAngle = missileData[0].data['g_lockAngleMax']
    angle = 0
    while angle < 360:
        maxLockcircleData.append([maxAngle*np.cos(np.radians(angle)),maxAngle*np.sin(np.radians(angle))])
        initLockCircleData.append([maxLockAngle*np.cos(np.radians(angle)),maxLockAngle*np.sin(np.radians(angle))])
        angle += 1
    # MaxAngle circle
    circleplot, = plot.plot(0,0,color='grey',linewidth=0.5)
    circleplot.set_data([o[0] for o in maxLockcircleData],
                        [o[1] for o in maxLockcircleData])
    # MaxLockAngle circle
    circleplot, = plot.plot(0,0,color='#d3d3d3',linewidth=0.5)
    circleplot.set_data([o[0] for o in initLockCircleData],
                        [o[1] for o in initLockCircleData])
    #==================================================================
    # 3d to 2d
    newTargetPath = []
    newFlaresPath = [ [] for i in range(len(flaresData)) ]

    for i in range(data.MaxFrame):
        m = missileData[i]
        t = targetData[i]
        rightVec = vec3.cross(m.upVec,m.fVec)
        diffVec = t.pVec - m.pVec
        calcRange = vec3.norm(diffVec)
        diffNormalized = vec3.normalize(diffVec)
        calcAngle = np.rad2deg(np.arccos(vec3.dot(diffNormalized,m.fVec)))

        if calcAngle <= 1.2*maxAngle and calcRange <= maxRange:
            newX = 90-np.rad2deg(np.arccos(vec3.dot(rightVec,diffNormalized))) #-90~90
            newY = 90-np.rad2deg(np.arccos(vec3.dot(m.upVec,diffNormalized)))-90  #-90~90
            newTargetPath.append([-newX,newY])
        else:
            if i == 0:
                newTargetPath.append([0,0])
            else:
                newTargetPath.append([newTargetPath[i-1][0],newTargetPath[i-1][1]])
        
        for fi in range(len(flaresData)):
        #for obj in flaresData:
            fl = flaresData[fi][i]
            diffVec = fl.pVec - m.pVec
            calcRange = vec3.norm(diffVec)
            diffNormalized = vec3.normalize(diffVec)
            calcAngle = np.rad2deg(np.arccos(vec3.dot(diffNormalized,m.fVec)))

            if calcAngle <= 1.2*maxAngle and calcRange <= maxRange:
                newX = 90-np.rad2deg(np.arccos(vec3.dot(rightVec,diffNormalized))) #-90~90
                newY = 90-np.rad2deg(np.arccos(vec3.dot(m.upVec,diffNormalized))) #-90~90
                newFlaresPath[fi].append([-newX,newY])
            else:
                newFlaresPath[fi].append([0,data.INFINITE])
    #==================================================================
    lockCirclePlot, = plot.plot(0,0,color='red',linewidth=0.5)
    lockCircleData = []
    for i in range(data.MaxFrame):
        m = missileData[i]
        t = targetData[i]
        circleData = []
        rightVec = vec3.cross(m.upVec,m.fVec)
        diffVec = m.sVec
        diffNormalized = diffVec#.normalize()
        x = 90-np.rad2deg(np.arccos(vec3.dot(rightVec,diffNormalized))) #-90~90
        y = 90-np.rad2deg(np.arccos(vec3.dot(m.upVec,diffNormalized)))  #-90~90
        angle = 0
        while angle < 360:
            circleData.append((-x-maxFOV*np.cos(np.radians(angle)),y+maxFOV*np.sin(np.radians(angle))))
            angle += 1
        lockCircleData.append([circleData])
    #-----------------------------------------------------------
    # list of objects to Plot
    ## PlotObjects related to Target Object
    color = 'blue'
    targetObjects = [
            plot.text(0,0,targetData[0].data['name'], color=color, size='x-small'),
            plot.plot(0,0,color=color,linewidth=0.5),
            newTargetPath,
            targetData]

    color = 'purple'
    flaresObjects = [
                    [
                        plot.text(0,0,'*', color=color, size='x-large'),
                        newFlaresPath[i]
                    ] for i in range(len(newFlaresPath))
                    ]
    lockCircle = [lockCirclePlot,lockCircleData]
    return [targetObjects,missileData,flaresObjects, lockCircle]
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

def genTextStr(m,t):
    """
    generates string to be used by matplotlib text

    Args:
        m (mObjects.MissileObject) : Missile Object
        t (mObjects.TargetObject) : Target Object
    Returns:
        string : text
    """
    return t.data['name'] + '\n' + str(round(vec3.norm(m.pVec-t.pVec)/1000,1))+'km'

def update(frame,simPlots):
    """
    update of plot, per frame

    Args:
        frame (float) : frame(time)
        simPlots (list of object data) : packed objects to be updated
    """
    targetObjects,missileData,flaresObjects, lockCircle = simPlots
    #------------------------------------------
    # [[textplot, data],[textplot, data],...]
    textplot,lineplot, obj, targetData = targetObjects

    lineplot[0].set_data([o[0] for o in obj[:frame+1]],
                         [o[1] for o in obj[:frame+1]])
    textplot.set_position((obj[frame][0],obj[frame][1]))
    textplot.set_text(genTextStr(missileData[frame],targetData[frame]))
    for textplot, obj in flaresObjects:
        textplot.set_position((obj[frame][0],obj[frame][1]))
    
    lockCirclePlot, lockCircleData = lockCircle
    lockCirclePlot.set_data([m[0] for o in lockCircleData[frame][:360] for m in o ], 
                            [m[1] for o in lockCircleData[frame][:360] for m in o ])
    
    #------------------------------------------
    ## do-things at end of frame
    if frame == data.MaxFrame-1:
        resetColor(targetObjects[0],'target')
        for flareObj in flaresObjects:
            resetColor(flareObj[0],'flare')