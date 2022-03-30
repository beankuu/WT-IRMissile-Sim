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

import sim1,sim2,sim3,sim4

#===============================================
#-----------------------------------------------

target =  mobj.TargetObject(data=data.target1,    pVec = vec3(2500,0,3000),  vVec=vec3(280, 0, 0) )
missile = mobj.MissileObject(data=data.missile2,  pVec = vec3(0,0,3000),     sVec=vec3(2500,0,3000) )

flareTypeData = data.flare0 if target.data['flareType'] == 0 else data.flare1
flareTimes = [0.1,0.3,0.5, 3.1,3.3,3.5, 6.1,6.3,6.5]
#flareTimes = []
flares = [ mobj.FlareObject(data=flareTypeData) for i in range(len(flareTimes)) ]

## list of [list of [SimObjects per frame]]...
targetData, missileData, flaresData = pathGen.genPaths(target,missile,[flares,flareTimes])

#==================================================================

fig = plt.figure()
fig.set_figwidth(10)
fig.set_figheight(6.18)

asim1 = fig.add_subplot(221,projection = '3d')
asim2 = fig.add_subplot(222)
asim3 = fig.add_subplot(223)
asim4 = fig.add_subplot(224)
#fig, ((asim1,asim2),(asim3,asim4)) = plt.subplots(2, 2, projection = '3d')
#trajectoryPlot = fig.add_subplot(projection = '3d')

sim1Plots = sim1.init(asim1, [targetData,missileData,flaresData])
sim2Plots = sim2.init(asim2, [missileData])
sim3Plots = sim3.init(asim3, [targetData,missileData,flaresData])
sim4Plots = sim4.init(asim4, [targetData, missileData])


#! update function
def update(frame,sim1Plots,sim2Plots,sim3Plots,sim4Plots):
    sim1.update(frame,asim1,sim1Plots)
    sim2.update(frame,asim2,sim2Plots)
    sim3.update(frame,asim3,sim3Plots)
    sim4.update(frame,asim4,sim4Plots)
    
#---------------------------------------------------------------------------
animation = ani.FuncAnimation(
    fig = fig,
    func = update,
    fargs = (sim1Plots,sim2Plots,sim3Plots,sim4Plots),
    frames = data.MaxFrame,
    interval = 1000/data.FPS
)
#FFwriter=ani.FFMpegWriter(fps=15, extra_args=['-vcodec', 'libx264'])
FFwriter=ani.FFMpegWriter(fps=30)
#animation.save('test.mp4',writer = FFwriter)
plt.show()
