from PyKDL import Joint,Rotation,Vector,Frame,Segment
from numpy import pi,array


finger_kin= [
    #Base joint
    Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(pi/2), Vector(0.0, 0.0 , 0.0))),
    #Proximal
    Segment(Joint(Joint.RotZ),
            Frame(Vector(0.0678, 0, 0.0))),
    #Distal1
    Segment(Joint(Joint.RotZ),
            Frame(Vector(0.03, 0, 0))),
    #Distal2
    Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotY(pi/2)*Rotation.RotZ(pi/2), Vector(0.0295, 0.0, 0.0)))
]

pos_sensor_proximal1=2.63869565217391304348*0.01
pos_sensor_proximal1=2.45*0.01 
pos_sensor_proximal2=2.21478260869565217392*0.01
pos_sensor_distal=.81521739130434782608*0.01
pos_sensor_distal=.8*0.01
dist_proximal_distal1=6.78*0.01
dist_distal1_distal2=3.0*0.01
dist_end_tip=1.35*0.01
dist_distal2_tip=2.95*0.01+dist_end_tip

torque_factors=[1.002,0.9,.49218]

#with sensors
finger_kin= [
    #Base joint
    Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(pi/2))),
    #Proximal
    Segment(Joint(Joint.RotZ),
            Frame(Vector(pos_sensor_proximal1, 0, 0.0))),
    #torque sensor "base"
    Segment(Joint(Joint.RotY),
            Frame()),
    #torque sensor "proximal"
    Segment(Joint(Joint.RotZ),
            Frame(Vector(dist_proximal_distal1-pos_sensor_proximal1, 0.0, 0.0))),
    #Distal1
    Segment(Joint(Joint.RotZ),
            Frame(Vector(dist_distal1_distal2, 0, 0))),
    #Distal2
    Segment(Joint(Joint.RotZ),
            Frame(Vector(pos_sensor_distal, 0.0, 0.0))),
    #torque sensor distal
    Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotY(pi/2)*Rotation.RotZ(pi/2), Vector(dist_distal2_tip-pos_sensor_distal, 0.0, 0.0)))
]

motor_joints=[0,1,4,5]
sensor_joints=[2,3,6]

total_finger_joints=len(motor_joints)+len(sensor_joints)

motor_locked_joints_temp=[False]*total_finger_joints

for i in xrange(total_finger_joints):
    if i in sensor_joints:
        motor_locked_joints_temp[i]=True
motor_locked_joints=[[False]+motor_locked_joints_temp]+[motor_locked_joints_temp]*3 #for all fingers

sensor_locked_joints=[]
for finger in motor_locked_joints:
    sensor_locked_joints.append([not(i) for i in finger])

thumb_finger_coupling = array([[1.0, 0.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0, 0.0],
                         [0.0, 0.0, 1.0, 0.0],
                         [0.0, 0.0, 0.0, 1.0],
                         [0.0, 0.0, 0.0, 1.0]])

sensor_finger_coupling = array([[1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0]])

fingers_coupling = [#thumb
    thumb_finger_coupling,
    #first
    thumb_finger_coupling[1:,1:],
    #middle
    thumb_finger_coupling[1:,1:],
    #ring
    thumb_finger_coupling[1:,1:]]
    

angle1=2.03357
angle2=1.96569
angle3=55.0

hands_kin = {
    'right' : [[#thumb
    Segment(Joint(Joint.None),
            Frame(Vector(-0.003,0.0271,0.0))),
    Segment(Joint(Joint.RotZ,-1),
            Frame(Rotation.RotX(angle3*pi/180.0)*Rotation.RotZ(90.0*pi/180.0),Vector(-0.009,0.114,0.097)))]+finger_kin,
               [#first
    Segment(Joint(Joint.None),
            Frame(Rotation.EulerZYZ(pi,-90.0*pi/180.0,-angle1*pi/180.0),Vector(-0.0043,0.040165,0.14543)))]+finger_kin,
               [#middle
    Segment(Joint(Joint.None),
            Frame(Rotation.EulerZYZ(pi,-90.0*pi/180.0,0.0),Vector(-0.0043,0.0,0.15015)))]+finger_kin,
               [#ring
    Segment(Joint(Joint.None),
            Frame(Rotation.EulerZYZ(pi,-90.0*pi/180.0,angle2*pi/180.0),Vector(-0.0043,-0.040165,0.14543)))]+finger_kin
               ],
    'left' : [[#thumb
    Segment(Joint(Joint.None),
            Frame(Vector(-0.003,-0.0271,0.0))),
    Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(-angle3*pi/180.0)*Rotation.RotZ(-90.0*pi/180.0),Vector(-0.009,-0.114,0.097)))]+finger_kin,
               [#first
    Segment(Joint(Joint.None),
            Frame(Rotation.EulerZYZ(0.0, 90.0*pi/180.0, -(pi-(angle2*pi/180.0))),Vector(-0.0043,-0.040165,0.14543)))]+finger_kin,
               [#middle
    Segment(Joint(Joint.None),
            Frame(Rotation.EulerZYZ(pi,-90.0*pi/180.0,0.0),Vector(-0.0043,0.0,0.15015)))]+finger_kin,
               [#ring
    Segment(Joint(Joint.None),
            Frame(Rotation.EulerZYZ(0.0, 90.0*pi/180.0,(pi-angle1*pi/180.0)),Vector(-0.0043,0.040165,0.14543)))]+finger_kin
               ]
    }

finger_limits=[[-15.0*pi/180.0,15.0*pi/180.0],
        [0.0*pi/180.0, 55.0*pi/180.0],
        [5.0*pi/180.0, 55.0*pi/180.0]]

fingers_lim = [#thumb
    [[0.0*pi/180.0,90.0*pi/180.0]]+finger_limits,
               #first
    finger_limits,
               #middle
    finger_limits,
               #ring
    finger_limits]

finger=2
segments=hands_kin['left'][finger]
#print "Segments"
print segments
finger_lim=fingers_lim[finger]
finger_lim_roboview=[[-15.0*pi/180.0,15.0*pi/180.0],
                     [0.0*pi/180.0, 90.0*pi/180.0],
                     [0*pi/180.0,0*pi/180.0],
                     [0*pi/180.0,0*pi/180.0],
                     [0.0*pi/180.0, 90.0*pi/180.0],
                     [0.0*pi/180.0, 90.0*pi/180.0],
                     [0*pi/180.0,0*pi/180.0]]

limits_min=[i[0] for i in finger_lim_roboview]
limits_max=[i[1] for i in finger_lim_roboview]
print "Limits"
print limits_min
print limits_max
