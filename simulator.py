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
#-----------------------------------------------

target =  mobj.TargetObject(data=data.target1,    pVec = vec3(3000,0,3000),  vVec=vec3(280, 0, 0),  isAfterburnerOnAt=[(0.0,6.0)] )
missile = mobj.MissileObject(data=data.missile1,  pVec = vec3(0,0,3000),     sVec=vec3(3000,0,3000) )

flareTypeData = data.flare0 if target.data['flareType'] == 0 else data.flare1

def periodicFlare(start,flareSeries,flaresSeriesPeriod,flarePeriod):
    periodicResult = []
    totalSeries = int((data.TIMEMAX-start)//flaresSeriesPeriod)
    for i in range(totalSeries):
        for j in range(flareSeries):
            periodicResult.append(start+i*flaresSeriesPeriod+j*flarePeriod)
    return periodicResult

#flareTimes = []
#flareTimes = [3.1,3.3,3.5, 6.1,6.3,6.5]
flareTimes = periodicFlare(1.0, 3, 1.0, 0.1)
print('flare @ ',flareTimes)

flares = [ mobj.FlareObject(data=flareTypeData) for i in range(len(flareTimes)) ]

## list of [list of [SimObjects per frame]]...
targetData, missileData, flaresData = pathGen.genPaths(target,missile,[flares,flareTimes])

#==================================================================

fig = plt.figure()
fig.set_figwidth(11)
fig.set_figheight(11*3/5)

grid = plt.GridSpec(3,5, wspace=0.4, hspace=0.3)
asim1 = fig.add_subplot(grid[:3,:3],projection = '3d')
asim2 = fig.add_subplot(grid[:2,3:5])
asim3 = fig.add_subplot(grid[2,3])
asim4 = fig.add_subplot(grid[2,4])
asim5 = fig.add_subplot(grid[1,4]) # seeker zoomin

sim1Plots = sim1.init(asim1, [targetData,missileData,flaresData])
sim2Plots = sim2.init(asim2, [targetData,missileData,flaresData])
sim3Plots = sim3.init(asim3, [missileData])
sim4Plots = sim4.init(asim4, [targetData, missileData])
sim5Plots = sim5.init(asim5, [targetData,missileData,flaresData])

#! update function
def update(frame):
    sim1.update(frame,asim1,sim1Plots)
    sim2.update(frame,asim2,sim2Plots)
    sim3.update(frame,asim3,sim3Plots)
    sim4.update(frame,asim4,sim4Plots)
    sim5.update(frame,asim5,sim5Plots)
    
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
