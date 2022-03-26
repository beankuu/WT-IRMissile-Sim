# calculation helper functions
import numpy as np
#import mObject.Vec3D as vec3
import data

# obj.Vec3D{P1,P2,P3}, a = dv/dt
#def calcAccelVec(P1,P2,P3,dt):
#    V1_2 = (P1-P2)/dt
#    V2_3 = (P2-P3)/dt
#    return (V1_2-V2_3)/(dt)
    #return np.linalg.norm(P3-P1)/(dt*dt)
#def calcGForce(acceleration, gravity):
#    return round((acceleration-[0,0,-gravity]).norm() / gravity,1)
# velocity(m/s), height(m)
def calcMach(velocity, height):
    #http://www.aerospaceweb.org/question/atmosphere/q0126.shtml
    # mach = velocity / sound_speed_at_given_altitude
    # sound_speed_at_given_altitude = 331 + 0.6*temperature
    speed_of_sound = 331 + 0.6*data.getTemperatureAtHeight(height)
    return velocity/speed_of_sound
# used...?
# obj.Vec3D{P2,P1}, v = dx/dt
#def calcSpeed(P2,P1,dt):
#    return (P2-P1).norm/dt