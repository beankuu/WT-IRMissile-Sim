import numpy as np

# customs
import data
import calculate as calc
import guidance as guid
import mObjects as obj

#*----------------------------------------------------------------------
def genTargetTrajectory():
    r,rho,theta = data.INIT_TARGET_HORIZ_COORDINATE
    #horizontal -> cartesian coordinate
    rotated_position = np.array([   r*np.cos(theta),
                                    r*np.cos(rho)*np.sin(theta),
                                    r*np.sin(rho)*np.sin(theta)
                                ])
    # +initial position
    TARGET_POSITION = data.INIT_MISSILE_POSITION+rotated_position
    #INIT_TARGET_POSITION = [x+y for x,y in zip(INIT_MISSILE_POSITION,TARGET_POSITION)]
    
    direction_vec = np.array([1.0, 0.0, 0.0])
    newdirection = direction_vec  

    # fly straight, until reaction time
    reaction_time = 0.5 #s
    f = 0
    velocity_per_frame = data.TARGET_INIT_VELOCITY/data.FPS
    while f < reaction_time*data.FPS:
        TARGET_POSITION += np.array([velocity_per_frame,0,0])
        c = []
        for ai in TARGET_POSITION: c.append(ai)
        for bi in newdirection: c.append(bi)
        yield np.array(c)
        #yield np.array(TARGET_POSITION)
        f += 1
    
    def rotate_vec_x(P,angle):
        return np.array([P[0],
                        np.cos(angle)*P[1]-np.sin(angle)*P[2],
                        np.sin(angle)*P[1]+np.cos(angle)*P[2]])
    def rotate_vec_y(P,angle):
        return np.array([np.cos(angle)*P[0]+np.sin(angle)*P[2],
                        P[1],
                        -np.sin(angle)*P[0]+np.cos(angle)*P[2]])
    def rotate_vec_z(P,angle):
        return np.array([np.cos(angle)*P[0]-np.sin(angle)*P[1],
                        np.sin(angle)*P[0]+np.cos(angle)*P[1],
                        P[2]])

    # 2. rotate
    maxrotation = 1.5
    rotation_radius = 1 #m

    downwards_degree = np.radians(30)

    angle = 0
    angle_per_rotate = maxrotation*2*np.pi/(data.MaxFrame-reaction_time*data.FPS)
    
    while f < data.MaxFrame:
        #angular_vector = vec_normalize(np.array([1.0,np.cos(angle),np.sin(angle)]))
        newdirection = direction_vec + np.array([0,rotation_radius,0])
        newdirection = rotate_vec_x(newdirection,angle)
        newdirection = rotate_vec_y(newdirection,downwards_degree)
        #newdirection = vec_normalize(direction_vec+rotation_radius*angular_vector)
        newdirection = calc.vec_normalize(newdirection)

        accel_gravity = data.getGravity(TARGET_POSITION[2])
        velocity_per_frame += accel_gravity/(data.FPS*data.FPS)
        if velocity_per_frame > data.t['maxspeed']/data.FPS:
            velocity_per_frame = data.t['maxspeed']/data.FPS
        
        TARGET_POSITION += velocity_per_frame*newdirection
        #print(np.concatenate((TARGET_POSITION,newdirection)))
        c = []
        for ai in TARGET_POSITION: c.append(ai)
        for bi in newdirection: c.append(bi)
        yield np.array(c)

        angle += angle_per_rotate
        f += 1
#*----------------------------------------------------------------------
def genMissileTrajectory():
    #INIT_MISSILE_POSITION
    """
    f = 0
    while f < data.MaxFrame:
        yield guidance.getMissileLocation(f/data.FPS)
        f += 1
    """
    phi = 0
    while phi < 2*np.pi:
        yield np.array([np.cos(phi), np.sin(phi), phi, True])
        phi += 2*np.pi/data.MaxFrame
    #"""
