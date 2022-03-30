## up right 2D graph, simulating PID's intgK value by time
import mObjects as mobj
import data

def init(plot,datapack):
    missileData = datapack[0]
    k_iData = [elm.intgK for elm in missileData]
    plot.set_xlim([0,data.MaxFrame])
    plot.set_ylim([0,1])

    plot.set_xlabel('시간(frame)')
    plot.set_ylabel('K_i(유도 적분상수)')

    #==================================================================
    lineplot, = plot.plot(0,0)
    return [k_iData,lineplot]
#-----------------------------------------------------------
def update(frame,ax,simPlots):
    k_iData, lineplot = simPlots
    lineplot.set_data([range(frame)],k_iData[:frame])
    