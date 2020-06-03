from __future__ import division
import pybullet as p
import pybullet_data

class MotomanRobot(object):
    ### Define the robot
    def __init__(self, server):
        ### get the server
        self.planningServer = server[0]
        self.executingServer = server[1]
        ### collect geometries from the robot
        self.known_geometries_planning = []
        self.known_geometries_executing = []
        ### get the urdf
        self.motomanGEO_p = p.loadURDF("motoman.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION, physicsClientId=self.planningServer)
        self.motomanGEO_e = p.loadURDF("motoman.urdf", useFixedBase=True, physicsClientId=self.executingServer)
        self.known_geometries_planning.append(self.motomanGEO_p)
        self.known_geometries_executing.append(self.motomanGEO_e)
        
        ### reset the base of motoman
        self.BasePosition = [0, 0, 0]
        self.BaseOrientation = [0, 0, 0, 1]
        ### set motoman home configuration
        self.homeConfiguration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ### update its current configuration
        self.updateCurrConfig(self.homeConfiguration)

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


    def resetConfiguration(self, resetConfiguration, robotGEO, server):
        for j in range(1, 8):
            p.resetJointState(robotGEO, j, resetConfiguration[j-1], physicsClientId=server)
        for j in range(11, 18):
            p.resetJointState(robotGEO, j, resetConfiguration[j-4], physicsClientId=server)


    def moveSingleArm(self, singleArmConfiguration, robotGEO, handType, server):
        if handType == "Left":
            for j in range(1, 8):
                p.resetJointState(robotGEO, j, singleArmConfiguration[j-1], physicsClientId=server)
            for j in range(11, 18):
                p.resetJointState(robotGEO, j, self.currConfiguration[j-4], physicsClientId=server)
        else:
            for j in range(1, 8):
                p.resetJointState(robotGEO, j, self.currConfiguration[j-1], physicsClientId=server)
            for j in range(11, 18):
                p.resetJointState(robotGEO, j, singleArmConfiguration[j-11], physicsClientId=server)
        p.stepSimulation(server)        


    def moveDualArm(self, armConfiguration, robotGEO, handType, server):
        if handType == "Left":
            for j in range(1, 8):
                p.resetJointState(robotGEO, j, armConfiguration[j-1], physicsClientId=server)
            for j in range(11, 18):
                p.resetJointState(robotGEO, j, armConfiguration[j-4], physicsClientId=server)
        else:
            for j in range(1, 8):
                p.resetJointState(robotGEO, j, armConfiguration[j-1], physicsClientId=server)
            for j in range(11, 18):
                p.resetJointState(robotGEO, j, armConfiguration[j-4], physicsClientId=server)
        p.stepSimulation(server)


    def updateCurrConfig(self, currConfiguration):
        self.currConfiguration = currConfiguration
        self.leftArmCurrConfiguration = self.currConfiguration[0:7]
        self.rightArmCurrConfiguration = self.currConfiguration[7:14]


    def printRobotJointInfo(self):
        ### This printing function is for debugging purpose (not used in experiment)
        ############################### information related to Motoman arm (joint info) ###################################
        print "Motoman Robot: " + str(self.motomanGEO_p)
        num_joints = p.getNumJoints(self.motomanGEO_p, self.planningServer)
        print "Num of joints: " + str(num_joints)
        for i in range(num_joints):
          print(p.getJointInfo(self.motomanGEO_p, i, self.planningServer))



