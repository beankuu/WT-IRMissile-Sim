## down right 2D graph, missile velocity
import mObjects as mobj
import data

def init(plot,datapack):
    targetData, missileData = datapack
    missile_speedData = [round(elm.vVec.norm()/0.2777,2) for elm in missileData]
    target_speedData = [round(elm.vVec.norm()/0.2777,2) for elm in targetData]
    plot.set_xlim([0,data.MaxFrame])
    plot.set_ylim([0,max(missile_speedData)+200])

    plot.set_xlabel('시간(frame)')
    plot.set_ylabel('속도(km/h)')

    #==================================================================
    missile_lineplot, = plot.plot(0,0,color='red')
    target_lineplot, = plot.plot(0,0,color='blue')
    return [missile_speedData,missile_lineplot, target_speedData,target_lineplot]
#-----------------------------------------------------------
def update(frame,ax,simPlots):
    data1,line1, data2,line2 = simPlots
    line1.set_data([range(frame)],data1[:frame])
    line2.set_data([range(frame)],data2[:frame])
    