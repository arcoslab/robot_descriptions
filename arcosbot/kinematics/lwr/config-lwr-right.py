from PyKDL import *
from math import pi
from lin_inter import a5a6_limit
from lin_inter import limit_kuka_to_negative
from lin_inter import fix_limit
from numpy import array

# main configuration
arm_type = 'lwr'
nJoints = 7

arm_instance = 'right'

# when set to False, the wlds solvers joint weights get modulated
# according to proximity to joint limits
disable_smart_joint_limit_avoidance = False

#yarp robot-arm port basename, must match the file name: config-arm-side
robotarm_portbasename= "/"+arm_type+"/"+arm_instance

# bridge communication
qin_portname = "/bridge/qin"
qcmded_portname = "/bridge/qcmded"
qcmd_portname = "/bridge/qcmd"

#inverse kinematics parameters

# low lambda (0.01): high tracking accuracy
# high lambda (0.5): high singularity clearance
ik_lambda = 0.10 # how much should singularities be avoided?

#TODO: these weights are not working right
ik_weightTS = (1.0,)*6  # how much should the cartesian goal directions be pursued?
ik_weightJS = (1.0,)*nJoints # how much should each joint be used to accomplish the goal?

speedScale=0.20

#max joint speed
max_vel=41.0*pi/180  # 30.0
#**************************for inertia this may change
max_vel=441.0*pi/180

#initial joint position
initial_joint_pos=[0.0,-1.2,0.7,1.4,0.35,-1.4,0.0]
#initial_vf_pose=["set", "goal", [-1., 0., 0., 0.8, 0., 0., 1., -0.15, 0., 1., 0., 1., 0., 0., 0., 1., 0.05]]

# arm configuration
arm_segments = [
        Segment(Joint(Joint.None),
            Frame(Rotation.RPY(0.71178, 0.85723, -0.71169),Vector(0.385121, -0.0570121, 1.15408)  )),
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
            Frame(Rotation.RotZ(3*pi/4)*Rotation.RotX(pi/2)*Rotation.RotY(pi), Vector(-0.075, -0.075, -0.094))),
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
arm_limits_fede = [[-169.5*pi/180, 169.5*pi/180],
              [-119.5*pi/180, -20.0*pi/180],
              [-169.5*pi/180, 169.5*pi/180],
              [-119.5*pi/180, -20.0*pi/180],
              [-169.5*pi/180, 169.5*pi/180],
              #[   0.5*pi/180, 119.9*pi/180],
              #[   5.0*pi/180, 169.9*pi/180]]
              [-119.5*pi/180, 119.5*pi/180],
              [-169.5*pi/180, 169.5*pi/180]]
arm_limits=arm_limits_default
#Extra margins in the limits for limit avoidance. This is necessary for all robots to define
arm_extralimit_margin=1.0*pi/180
arm_extralimits=[[x[0]+arm_extralimit_margin,x[1]-arm_extralimit_margin] for x in arm_limits]

#control loop speed in seconds
rate=0.01

#special A5 and A6 joint limits
limitsA6=[  #table for limits on A6 when changing A5. Data: A5_angle, A6minlim, A6maxlim
[ -130,  -114,    90],
[ -120,  -130,    95],
[ -100,  -138,    84],
[  -80,  -140,    90],
[  -60,  -140,    96],
[  -50,  -138,   105],
[  -40,  -132,   130],
[  -35,  -132,   144],
[  -30,  -135,   155],
[  -25,  -150,   160],
[  -20,  -165,   170],
[  -10,  -170,   170],
[    0,  -170,   170],
[   10,  -170,   170],
[   20,  -170,   170],
[   30,   -70,   170],
[   40,   -27,   170],
[   50,   -12,   170],
[   60,    -5,   170],
[   70,    -3,   170],
[   80,    -4,   170],
[  100,    -3,   170],
[  120,     0,   170],
[  130,     0,   170]
]


limitsA6=[map(float,i) for i in limitsA6]
limitsA5= [ #table for limits on A5 when changing A6. Data: A6_angle, A5minlim, A5maxlim
[-170, -10, 20],
[-160, -21, 21],
[-135, -28, 22],
[-131, -117, 23],
[-113, -130, 25],
[-70,  -130, 30],
[-26, -130, 39],
[-11, -130, 50],
[-4, -130, 60],
[-2, -130, 70],
[0, -130, 130],
[82, -130, 130],
[83, -99, 130],
[95, -60, 130],
[104, -50, 130],
[154, -30, 130],
[160, -24, 130],
[169, -20, 130],
[170, -20, 130]
]
limitsA5=[map(float,i) for i in limitsA5]
#limitsA5=[[i[0],limit_kuka_to_negative(i[1]),limit_kuka_to_negative(i[2])] for i in limitsA5]
limitsA5=[fix_limit(i) for i in limitsA5]

#Some safety distance to the hand limits
limitsA5=[[i[0], i[1]+5, i[2]-5] for i in limitsA5]
limitsA6=[[i[0], i[1]+5, i[2]-5] for i in limitsA6]

#print "Limits Table", limitsA5

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
