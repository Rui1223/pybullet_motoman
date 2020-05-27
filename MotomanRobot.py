from __future__ import division
import pybullet as p
import pybullet_data

class MotomanRobot(object):
    ### Define the robot
    def __init__(self, server):
        ### get the server
        self.planningServer = server[0]
        self.executingServer = server[1]
        ### get the urdf
        self.motomanGEO_p = p.loadURDF("motoman.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION, physicsClientId=self.planningServer)
        self.motomanGEO_e = p.loadURDF("motoman.urdf", useFixedBase=True, physicsClientId=self.executingServer)
        
        ### reset the base of motoman
        self.BasePosition = [0, 0, 0]
        self.BaseOrientation = [0, 0, 0, 1]
        ### set motoman home configuration
        self.homeConfiguration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        ### joint and end effector information
        ### end-effector index
        self.left_ee_idx = 10 ### left hand ee
        self.right_ee_idx = 20 ### right hand ee
        ### There is a torso joint which connects the lower and upper body (-2.957 ~ 2.957)
        ### But so far we decide to make that torso joint fixed
        ### For each arm, there are 10 joints and 7 of them are revolute joints
        ### There are total 14 revolute joints for each arm
        ### lower limits for null space
        self.ll = [-3.13, -1.90, -2.95, -2.36, -3.13, -1.90, -3.13, -3.13, -1.90, -2.95, -2.36, -3.13, -1.90, -3.13]
        ### upper limits for null space
        self.ul = [3.13, 1.90, 2.95, 2.36, 3.13, 1.90, 3.13, 3.13, 1.90, -2.95, 2.36, 3.13, 1.90, 3.13]
        ### joint ranges for null space
        self.jr = [6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26, 6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26]
        ### restposes for null space
        self.rp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]



    def printRobotJointInfo(self):
        ### This printing function is for debugging purpose (not used in experiment)
        ############################### information related to Motoman arm (joint info) ###################################
        print "Motoman Robot: " + str(self.motomanGEO_p)
        num_joints = p.getNumJoints(self.motomanGEO_p, self.planningServer)
        print "Num of joints: " + str(num_joints)
        for i in range(num_joints):
          print(p.getJointInfo(self.motomanGEO_p, i, self.planningServer))



