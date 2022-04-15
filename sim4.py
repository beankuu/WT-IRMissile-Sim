## down right 2D graph, missile velocity
import mObjects as mobj
from mObjects import Vec3D as vec3
import data

def init(plot,datapack):
    """
    initializing plot,
    generating 2d data to plot

    Args:
        plot (matplotlib.pyplot.subplot) : matplotlib plot to be drawn
        datapack ([targetData, missileData]) : [list of target, list of missile] data

    Returns:
        list: [list of speed per frame(missile), matplotlib line plot, list of speed per frame(target), matplotlib line plot]
    """
    targetData, missileData = datapack
    missile_speedData = [vec3.norm(elm.vVec)/0.2777 for elm in missileData]
    target_speedData = [vec3.norm(elm.vVec)/0.2777 for elm in targetData]
    
    plot.set_xlim([0,data.TIMEMAX])
    plot.set_ylim([0,max(missile_speedData)+200])

    plot.set_xlabel('시간(s)')
    plot.set_ylabel('속도(km/h)')

    # to right side
    plot.yaxis.tick_right()
    plot.yaxis.set_label_position("right")

    #==================================================================
    missile_lineplot, = plot.plot(0,0,color='red')
    target_lineplot, = plot.plot(0,0,color='blue')
    return [missile_speedData,missile_lineplot, target_speedData,target_lineplot]
#-----------------------------------------------------------
def update(frame,simPlots):
    """
    update of plot, per frame

    Args:
        frame (float) : frame(time)
        simPlots (list of object data) : packed objects to be updated
    """
    data1,line1, data2,line2 = simPlots
    timeList = [x * data.dt for x in range(0, frame)]
    line1.set_data(timeList,data1[:frame])
    line2.set_data(timeList,data2[:frame])
    