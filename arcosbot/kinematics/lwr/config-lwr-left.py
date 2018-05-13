from PyKDL import *
from math import pi
from lin_inter import a5a6_limit
from lin_inter import limit_kuka_to_positive
from lin_inter import fix_limit
from numpy import array

# main configuration
arm_type = 'lwr'
nJoints = 7

arm_instance= 'left'

# when set to False, the wlds solvers joint weights get modulated
# according to proximity to joint limits
disable_smart_joint_limit_avoidance = False

#yarp robot-arm port basename, must match the file name: config-arm-side
robotarm_portbasename= "/"+arm_type+"/"+arm_instance

# bridge communication
qin_portname = "/qin"
qcmded_portname = "/qcmded"
qcmd_portname = "/qcmd"

#inverse kinematics parameters

# low lambda (0.01): high tracking accuracy
# high lambda (0.5): high singularity clearance
ik_lambda = 0.10 # how much should singularities be avoided?

#TODO: these weights are not working right
ik_weightTS = (1.0,)*6  # how much should the cartesian goal directions be pursued?
ik_weightJS = (1.0,)*nJoints # how much should each joint be used to accomplish the goal?

#max joint speed
#max_vel=15.0*pi/180
max_vel=30.0*pi/180

#initial joint position
#initial_joint_pos=[0.78,1.6,-0.4,1.3,1.0,0.5,0.7]
#initial joint pos (open left position)
#initial_joint_pos = [0.380093, 0.383757, -0.475088, -1.868434, -1.302008, 1.671749, 0.112623]
#initial joint pos (grabbing forward position
#initial_joint_pos = [0.322857, 1.656332, 0.094038, -1.183425, -1.252687, 1.748051, 2.019311]
initial_joint_pos = [0.4, 1.15, -1., -1.7, -1., 1.4, 0.]


# arm configuration
arm_segments = [
        Segment(Joint(Joint.None),
            Frame(Rotation.RPY(0.71372, -0.84806, -2.42787),Vector(0.395,0.059,1.149))),
        Segment(Joint(Joint.None),
            Frame(Rotation.Identity(), Vector(0.0, 0.0, 0.11))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(-pi/2), Vector(0.0, 0.0, 0.20))),
        Segment(Joint(Joint.RotZ, -1),
            Frame(Rotation.RotX(pi/2), Vector(0.0, -0.20, 0.0))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(pi/2), Vector(0, 0, .20))),
        Segment(Joint(Joint.RotZ, -1),
            Frame(Rotation.RotX(-pi/2), Vector(0, 0.2, 0))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotX(-pi/2), Vector(0, 0, 0.19))),
        Segment(Joint(Joint.RotZ, -1),
            Frame(Rotation.RotX(pi/2), Vector(0, -0.078, 0.0))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.RotZ(pi*3/4)*Rotation.RotX(-pi/2)*Rotation.RotY(-pi/2), Vector(0.075, -0.075, -0.094))),
#        Segment(Joint(Joint.None),
#            Frame(Rotation.Identity(), Vector(0.07, -0.025, 0.28))),
            ]

arm_limits_default = [[-169.5*pi/180, 169.5*pi/180],
              [-119.5*pi/180, 119.5*pi/180],
              [-169.5*pi/180, 169.5*pi/180],
              [-119.5*pi/180, 119.5*pi/180],
              [-169.5*pi/180, 169.5*pi/180],
              [-119.5*pi/180, 119.5*pi/180],
              [-169.5*pi/180, 169.5*pi/180]]
arm_limits_fede = [[-169.9*pi/180, 169.9*pi/180],
              [-119.9*pi/180, -20.0*pi/180],
              [-169.9*pi/180, 169.9*pi/180],
              [-119.9*pi/180, -20.0*pi/180],
              [-169.9*pi/180, 169.9*pi/180],
              #[   0.5*pi/180, 119.9*pi/180],
              #[   5.0*pi/180, 169.9*pi/180]]
              [-119.9*pi/180, 119.9*pi/180],
              [-169.9*pi/180, 169.9*pi/180]]
arm_limits=arm_limits_default
#Extra margins in the limits for limit avoidance. This is necessary for all robots to define
arm_extralimit_margin=1.0*pi/180
arm_extralimits=[[x[0]+arm_extralimit_margin,x[1]-arm_extralimit_margin] for x in arm_limits]

#control loop speed in seconds
rate=0.01
fast_mode=False
virtual_rate=0.01
fast_rate=0.005

#special A5 and A6 joint limits
limitsA6=[#table for limits on A6 when changing A5. Data: A5_angle, A6minlim, A6maxlim
[ -130,  -170,     0],
[ -120,  -170,     0],
[ -100,  -170,     0],
[  -80,  -170,     0],
[  -60,  -170,     0],
[  -50,  -170,     4],
[  -40,  -170,    20],
[  -35,  -170,    47],
[  -30,  -170,    65],
[  -25,  -170,    77],
[  -20,  -170,    90],
[  -10,  -170,   110],
[    0,  -170,   170],
[   10,  -170,   170],
[   20,  -170,   170],
[   30,  -155,   142],
[   40,  -121,   140],
[   50,   -97,   140],
[   60,   -90,   133],
[   70,   -86,   133],
[   80,   -81,   138],
[  100,   -81,   138],
[  120,   -81,   125],
[  130,   -87,   112]
]
limitsA6=[map(float,i) for i in limitsA6]
limitsA5= [ #table for limits on A5 when changing A6. Data: A6_angle, A5minlim, A5maxlim
[-170, -130, 20],
[-153, -130, 29],
[-119, -130, 40],
[-98, -130, 50],
[-80, -130, 78],
[-81, -130, 130],
[-1, -130, 130],
[0, -55, 130],
[20, -40, 130], 
[64, -29, 130],
[108, -10, 130],
[110, -8, 130],
[125, -7, 120],
[132, -6, 58],
[142, -4, 30],
[155, -2, 25],
[170, 0, 20]
]
limitsA5=[map(float,i) for i in limitsA5]

#Some safety distance to the hand limits
limitsA5=[[i[0], i[1]+5, i[2]-5] for i in limitsA5]
limitsA6=[[i[0], i[1]+5, i[2]-5] for i in limitsA6]

#limitsA5=[[i[0],limitKukaToPositive(i[1]),limitKukaToPositive(i[2])] for i in limitsA5]
limitsA5=[fix_limit(i) for i in limitsA5]
print "Limits Table", limitsA5

def updateJntLimits(jnt_pos):
    a5angle=jnt_pos[5]
    a6angle=jnt_pos[6]
    a5limits,a6limits=a5a6_limit(a5angle*360/(2*pi),a6angle*360/(2*pi),limitsA5,limitsA6)
    joint_limits=arm_extralimits
    joint_limits[5]=map(lambda x: x*2*pi/360, a5limits)
    joint_limits[6]=map(lambda x: x*2*pi/360, a6limits)
    return joint_limits

def update_joint_weights(q,qdot):
        return array([1.0]*nJoints)


#for roboview
segments=arm_segments
limits_min=[i[0] for i in arm_limits]
limits_max=[i[1] for i in arm_limits]
