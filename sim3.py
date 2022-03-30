## down left 2D graph, Missile forward View
from cmath import pi
import mObjects as mobj
from mObjects import Vec3D as vec3
import data
import numpy as np

def init(plot,datapack):
    targetData,missileData,flaresData = datapack
    maxAngle = missileData[0].data['g_fov']/2
    maxRange =  missileData[0].data['rangeBand0']
    plot.set_xlim([-maxAngle*1.5,maxAngle*1.5])
    plot.set_ylim([-maxAngle*1.5,maxAngle*1.5])
    plot.set_xlabel('fov')
    plot.set_ylabel('fov')
    #=================================================================
    # range circle
    circleData = []
    angle = 0
    while angle < 360:
        circleData.append([maxAngle*np.cos(np.radians(angle)),maxAngle*np.sin(np.radians(angle))])
        angle += 1
    circleplot, = plot.plot(0,0,color='grey',linewidth=0.5)
    circleplot.set_data([o[0] for o in circleData],
                        [o[1] for o in circleData])
    #==================================================================
    # 3d to 2d
    newTargetPath = []
    newFlaresPath = [ [] for i in range(len(flaresData)) ]

    xymultiplier = maxAngle
    for i in range(data.MaxFrame):
        m = missileData[i]
        t = targetData[i]
        rightVec = m.upVec.cross(m.sVec)
        diffVec = t.pVec - m.pVec
        calcRange = diffVec.norm()
        diffNormalized = diffVec.normalize()
        calcAngle = np.rad2deg(np.arccos(diffNormalized.dot(m.sVec)))

        if calcAngle <= 1.5*maxAngle and calcRange <= maxRange:
            newX = 90-np.rad2deg(np.arccos(rightVec.dot(diffNormalized)))
            newY = 90-np.rad2deg(np.arccos(m.upVec.dot(diffNormalized)))
            newTargetPath.append([-newX,newY])
        else:
            if i == 0:
                newTargetPath.append([0,0])
            else:
                newTargetPath.append([newTargetPath[i-1][0],newTargetPath[i-1][1]])
        
        for fi in range(len(flaresData)):
        #for obj in flaresData:
            fl = flaresData[fi][i]
            diffVec = (fl.pVec - m.pVec)
            calcRange = diffVec.norm()
            diffNormalized = diffVec.normalize()
            calcAngle = np.rad2deg(np.arccos(diffNormalized.dot(m.sVec)))

            if calcAngle <= 1.5*maxAngle and calcRange <= maxRange:
                newX = 90-np.rad2deg(np.arccos(rightVec.dot(diffNormalized)))
                newY = 90-np.rad2deg(np.arccos(m.upVec.dot(diffNormalized)))
                newFlaresPath[fi].append([-newX,newY])
            else:
                newFlaresPath[fi].append([0,data.INFINITE])
    #==================================================================
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
    return [targetObjects,missileData,flaresObjects]
#-----------------------------------------------------------
def resetColor(resetObject,objType):
    if objType == 'target': color = 'blue'
    elif objType == 'missile': color = 'red'
    elif objType == 'flare': color = 'purple'
    else: color = 'grey'
    resetObject.set_color(color)

def genTextStr(m,t):
    return t.data['name'] + '\n' + str(round((m.pVec-t.pVec).norm()/1000,1))+'km'

def update(frame,ax,simPlots):
    targetObjects,missileData,flaresObjects = simPlots
    #------------------------------------------
    # [[textplot, data],[textplot, data],...]
    textplot,lineplot, obj, targetData = targetObjects

    lineplot[0].set_data([o[0] for o in obj[:frame+1]],
                         [o[1] for o in obj[:frame+1]])
    textplot.set_position((obj[frame][0],obj[frame][1]))
    textplot.set_text(genTextStr(missileData[frame],targetData[frame]))
    for textplot, obj in flaresObjects:
        textplot.set_position((obj[frame][0],obj[frame][1]))

    #------------------------------------------
    ## do-things at end of frame
    if frame == data.MaxFrame-1:
        resetColor(targetObjects[0],'target')
        for flareObj in flaresObjects:
            resetColor(flareObj[0],'flare')