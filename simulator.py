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

import sim1,sim2,sim3,sim4, sim5

#===============================================
def periodicFlare(start,flareSeries,flaresSeriesPeriod,flarePeriod):
    """
    generates list of flare period

    Args:
        start (float) : flare start time (s)
        flareSeries (int) : how many flares per series 
        flaresSeriesPeriod (float) : delta time between elements in series (s)
        flarePeriod (float) : delta time between series as a pack (s)
    Returns:
        list : list of flare periods (s)
    """
    periodicResult = []
    totalSeries = int((data.TIMEMAX-start)//flaresSeriesPeriod)
    for i in range(totalSeries):
        for j in range(flareSeries):
            periodicResult.append(start+i*flaresSeriesPeriod+j*flarePeriod)
    return periodicResult
#-----------------------------------------------

targetLocation = [ 
    vec3(2000,300,3000)
]

target =  mobj.TargetObject(data=data.target0,    pVec = targetLocation[0],  vVec=vec3(250, 0, 0),  isAfterburnerOnAt=[(0.0,6.0)] )
#missile = mobj.MissileObject(data=data.missile2,  pVec = vec3(0,0,3000),     sVec=targetLocation[0] )

missiles = [
    #mobj.MissileObject(data=data.missile0,  pVec = vec3(0,0,3000),     sVec=targetLocation[0] ),
    #mobj.MissileObject(data=data.missile1,  pVec = vec3(0,0,3000),     sVec=targetLocation[0] ),
    #mobj.MissileObject(data=data.missile2,  pVec = vec3(0,0,3000),     sVec=targetLocation[0] ),
    mobj.MissileObject(data=data.missile3,  pVec = vec3(0,0,3000),     sVec=targetLocation[0] ),
    #mobj.MissileObject(data=data.missile4,  pVec = vec3(0,0,3000),     sVec=targetLocation[0] )
]

flareTypeData = data.flare0 if target.data['flareType'] == 0 else data.flare1
flareTimes = periodicFlare(1.0, 2, 2.0, 0.1)
flareTimes = []
print('flare @ ',flareTimes)

flares = [ mobj.FlareObject(data=flareTypeData) for i in range(len(flareTimes)) ]

## list of [list of [SimObjects per frame]]...
#targetData, missileData, flaresData = pathGen.genPaths(target,missile,[flares,flareTimes])
targetData, missilesData, flaresData = pathGen.genPaths(target,missiles,[flares,flareTimes])

#==================================================================

fig = plt.figure()
fig.set_figwidth(11)
fig.set_figheight(11*3/5)

grid = plt.GridSpec(3,5, wspace=0.4, hspace=0.3)
asim1 = fig.add_subplot(grid[:3,:3],projection = '3d')
asim2 = fig.add_subplot(grid[:2,3:5])
asim3 = fig.add_subplot(grid[2,3])
asim4 = fig.add_subplot(grid[2,4])
asim5 = fig.add_subplot(grid[0,4]) # seeker zoomin

sim1Plots = sim1.init(asim1, [targetData,missilesData,flaresData])
sim2Plots = sim2.init(asim2, [targetData,missilesData[0],flaresData])
sim3Plots = sim3.init(asim3, [missilesData[0]])
sim4Plots = sim4.init(asim4, [targetData, missilesData[0]])
sim5Plots = sim5.init(asim5, [targetData,missilesData[0],flaresData])

#! update function
def update(frame):
    """
    root of update of plot, per frame

    Args:
        frame (float) : frame(time)
    """
    sim1.update(frame,sim1Plots)
    sim2.update(frame,sim2Plots)
    sim3.update(frame,sim3Plots)
    sim4.update(frame,sim4Plots)
    sim5.update(frame,sim5Plots)
    
#---------------------------------------------------------------------------
animation = ani.FuncAnimation(
    fig = fig,
    func = update,
    fargs = (),
    frames = data.MaxFrame,
    interval = 1000/data.FPS
)
#FFwriter=ani.FFMpegWriter(fps=15, extra_args=['-vcodec', 'libx264'])
FFwriter=ani.FFMpegWriter(fps=30)
#animation.save('test.mp4',writer = FFwriter)
plt.show()
