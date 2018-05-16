from PyKDL import *
from math import pi
from lin_inter import a5a6_limit
from lin_inter import limit_kuka_to_negative
from lin_inter import fix_limit
from numpy import array

# main configuration
arm_type = 'lwr'
nJoints = 12
torso_joints= []
torso_instance = "left"

arm_instance = 'right'

# when set to False, the wlds solvers joint weights get modulated
# according to proximity to joint limits
disable_smart_joint_limit_avoidance = False

#yarp robot-arm port basename, must match the file name: config-arm-side
robotarm_portbasename= "/"+arm_type+"/"+arm_instance

# bridge communication
torso_qin_portname = "/torso_qin"
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
max_vel=800.0*pi/180
speedScale=0.2
speedScale=0.6
#speedScale=1.0
speedScale=2.0

#initial joint position
initial_joint_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0,-1.2,0.7,1.4,0.35,-1.4,0.0]
#initial_vf_pose=["set", "goal", [-1., 0., 0., 0.8, 0., 0., 1., -0.15, 0., 1., 0., 1., 0., 0., 0., 1., 0.05]]
initial_joint_pos=[0.0, 0.1, 0.8, 0.8, 0.0, 0.0, -0.4, 0.0, 0.4, 0.0, -1.7, 0.0]

# joint_p_controller
jpctrl_kp=1.5
jpctrl_kp=4.5

# arm configuration
arm_segments = [
        Segment(Joint(Joint.None),
            Frame(Rotation.Identity(), Vector(0.4, 0.0, 1.15)  )),
        Segment(Joint(Joint.TransZ),
            Frame(Rotation.Identity(), Vector(0.0, 0.0, 0.0))),
        Segment(Joint(Joint.None),
            Frame(Rotation.RPY(0.0, 1.57, 0.0), Vector(0.0, 0.0, 0.0))),
        Segment(Joint(Joint.TransY, -1),
            Frame(Rotation.Identity(), Vector(0.0, 0.0, 0.0))),
        Segment(Joint(Joint.RotY, -1),
            Frame(Rotation.Identity(), Vector(0.0, 0.0, 0.0))),
        Segment(Joint(Joint.RotX),
            Frame(Rotation.Identity(), Vector(0.0, 0.0, 0.0))),
        Segment(Joint(Joint.RotZ),
            Frame(Rotation.Identity(), Vector(0.0, 0.0, 0.0))),
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
            Frame(Rotation.RotZ(-2*pi/4.0)*Rotation.RotX(-pi/2)*Rotation.RotY(-pi), Vector(-0.106, 0.0, -0.094))),
#        Segment(Joint(Joint.None),
#            Frame(Rotation.Identity(), Vector(0.07, -0.025, 0.28))),
            ]

arm_limits_default = [[-0.5, 0.5],
                      [0.1, 0.4],
                      [0.0*pi/180.0, 85*pi/180.0],
                      [0.0*pi/180.0, 90.0*pi/180.0],
                      [- 180.0*pi/180.0, 180.0*pi/180.0],
                      [-169.5*pi/180, 169.5*pi/180],
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

#Control-Loop speed in seconds
rate=0.01
fast_mode=False
virtual_rate=0.01
fast_rate=0.005

#special A5 and A6 joint limits
limitsA6=[  #table for limits on A6 when changing A5. Data: A5_angle, A6minlim, A6maxlim
[ -130,  -140,   140],
[  -90,  -140,   140],
[  -45,  -140,   140],
[  -20,  -140,   140],
[  -10,  -170,   170],
[    0,  -170,   170],
[   10,  -170,   170],
[   20,  -170,   170],
[   30,  -170,   170],
[   45,  -170,   170],
[   90,  -170,   170],
[  130,  -170,  -170]
]


limitsA6=[map(float,i) for i in limitsA6]
limitsA5= [ #table for limits on A5 when changing A6. Data: A6_angle, A5minlim, A5maxlim
    [-170, -130, 130],
    [ -90, -130, 130],
    [ 0.0, -130, 130],
    [  90, -130, 130],
    [ 170, -130, 130]
]
limitsA5=[map(float,i) for i in limitsA5]
#limitsA5=[[i[0],limit_kuka_to_negative(i[1]),limit_kuka_to_negative(i[2])] for i in limitsA5]
limitsA5=[fix_limit(i) for i in limitsA5]

#Some safety distance to the hand limits
limitsA5=[[i[0], i[1]+5, i[2]-5] for i in limitsA5]
limitsA6=[[i[0], i[1]+5, i[2]-5] for i in limitsA6]

#print "Limits Table", limitsA5

def updateJntLimits(jnt_pos):
    a5angle=jnt_pos[10]
    a6angle=jnt_pos[11]
    a5limits,a6limits=a5a6_limit(a5angle*360/(2*pi),a6angle*360/(2*pi),limitsA5,limitsA6)
    joint_limits=arm_extralimits
    joint_limits[-2]=map(lambda x: x*2*pi/360, a5limits)
    joint_limits[-1]=map(lambda x: x*2*pi/360, a6limits)
    return joint_limits

def update_joint_weights(q,qdot):
    return array([1.0]*nJoints)

#for roboview
segments=arm_segments
limits_min=[i[0] for i in arm_limits]
limits_max=[i[1] for i in arm_limits]
