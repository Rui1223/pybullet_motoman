from __future__ import division
import pybullet as p
import pybullet_data
import time

class MotomanRobot(object):
    ### Define the robot
    def __init__(self, servers):
        ### get the server
        self.planningServer = servers[0]
        self.executingServer = servers[1]
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
        self.updateLeftArmConfig(self.homeConfiguration[0:7], self.planningServer)
        self.updateRightArmConfig(self.homeConfiguration[7:14], self.planningServer)
        self.updateLeftArmConfig(self.homeConfiguration[0:7], self.executingServer)
        self.updateRightArmConfig(self.homeConfiguration[7:14], self.executingServer)        

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
        self.ul = [3.13, 1.90, 2.95, 2.36, 3.13, 1.90, 3.13, 3.13, 1.90, 2.95, 2.36, 3.13, 1.90, 3.13]
        ### joint ranges for null space
        self.jr = [6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26, 6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26]
        ### restposes for null space
        self.rp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # self.printRobotJointInfo()


    def moveSingleArm(self, singleArmConfiguration, handType):
        if handType == "Left":
            p.setJointMotorControlArray(self.motomanGEO_e, range(1, 8), controlMode=p.POSITION_CONTROL, 
                            targetPositions = singleArmConfiguration, physicsClientId=self.executingServer)
            self.updateLeftArmConfig(singleArmConfiguration, self.executingServer)
        else:
            p.setJointMotorControlArray(self.motomanGEO_e, range(11, 18), controlMode=p.POSITION_CONTROL, 
                            targetPositions = singleArmConfiguration, physicsClientId=self.executingServer)
            self.updateRightArmConfig(singleArmConfiguration, self.executingServer)

        # p.stepSimulation(physicsClientId=self.executingServer)

        # for i in range(0,10):
        #     p.stepSimulation(physicsClientId=self.executingServer)
        #     time.sleep(1/240.0)


    def moveSingleArm_resetState(self, singleArmConfiguration, handType):
        if handType == "Left":
            for j in range(1, 8):
                p.resetJointState(self.motomanGEO_e, j, singleArmConfiguration[j-1], physicsClientId=self.executingServer)
            # for j in range(11, 18):
            #     p.resetJointState(self.motomanGEO_e, j, self.rightArmCurrConfiguration_e[j-11], physicsClientId=self.executingServer)

            self.updateLeftArmConfig(singleArmConfiguration, self.executingServer)

        else:
            # for j in range(1, 8):
            #     p.resetJointState(self.motomanGEO_e, j, self.leftArmCurrConfiguration_e[j-1], physicsClientId=self.executingServer)
            for j in range(11, 18):
                p.resetJointState(self.motomanGEO_e, j, singleArmConfiguration[j-11], physicsClientId=self.executingServer)

            self.updateRightArmConfig(singleArmConfiguration, self.executingServer)




    def moveDualArm(self, dualArmConfiguration):
        p.setJointMotorControlArray(self.motomanGEO_e, range(1, 8), controlMode=p.POSITION_CONTROL, 
                        targetPositions = dualArmConfiguration[0:7], physicsClientId=self.executingServer)
        p.setJointMotorControlArray(self.motomanGEO_e, range(11, 18), controlMode=p.POSITION_CONTROL, 
                        targetPositions = dualArmConfiguration[7:14], physicsClientId=self.executingServer)
        self.updateLeftArmConfig(dualArmConfiguration[0:7], self.executingServer)
        self.updateRightArmConfig(dualArmConfiguration[7:14], self.executingServer)

        # p.stepSimulation(physicsClientId=self.executingServer)

        # for i in range(0,10):
        #     p.stepSimulation(physicsClientId=self.executingServer)
        #     time.sleep(1/240.0)

    def moveDualArm_resetState(self, dualArmConfiguration):
        for j in range(1, 8):
            p.resetJointState(self.motomanGEO_e, j, dualArmConfiguration[j-1], physicsClientId=self.executingServer)
        for j in range(11, 18):
            p.resetJointState(self.motomanGEO_e, j, dualArmConfiguration[j-4], physicsClientId=self.executingServer)

        self.updateLeftArmConfig(dualArmConfiguration[0:7], self.executingServer)
        self.updateRightArmConfig(dualArmConfiguration[7:14], self.executingServer)



    def setSingleArmToConfig(self, singleArmConfig, handType):
        if handType == "Left":
            for j in range(1, 8):
                p.resetJointState(self.motomanGEO_p, j, singleArmConfig[j-1], physicsClientId=self.planningServer)
            for j in range(11, 18):
                p.resetJointState(self.motomanGEO_p, j, self.rightArmCurrConfiguration_p[j-11], physicsClientId=self.planningServer)
        else:
            for j in range(1, 8):
                p.resetJointState(self.motomanGEO_p, j, self.leftArmCurrConfiguration_p[j-1], physicsClientId=self.planningServer)
            for j in range(11, 18):
                p.resetJointState(self.motomanGEO_p, j, singleArmConfig[j-11], physicsClientId=self.planningServer)

        p.stepSimulation(physicsClientId=self.planningServer)



    def setDualArmToConfig(self, dualArmConfig):
        for j in range(1, 8):
            p.resetJointState(self.motomanGEO_p, j, dualArmConfig[j-1], physicsClientId=self.planningServer)
        for j in range(11, 18):
            p.resetJointState(self.motomanGEO_p, j, dualArmConfig[j-4], physicsClientId=self.planningServer)

        p.stepSimulation(physicsClientId=self.planningServer)



    def resetConfig(self, resetConfiguration):
        for j in range(1, 8):
            p.resetJointState(self.motomanGEO_p, j, resetConfiguration[j-1], physicsClientId=self.planningServer)
        for j in range(11, 18):
            p.resetJointState(self.motomanGEO_p, j, resetConfiguration[j-4], physicsClientId=self.planningServer)

        p.stepSimulation(physicsClientId=self.planningServer)



    def updateLeftArmConfig(self, currLeftArmConfig, server):
        if server == self.planningServer:
            self.leftArmCurrConfiguration_p = currLeftArmConfig
        else:
            self.leftArmCurrConfiguration_e = currLeftArmConfig


    def updateRightArmConfig(self, currRightArmConfig, server):
        if server == self.planningServer:
            self.rightArmCurrConfiguration_p = currRightArmConfig
        else:
            self.rightArmCurrConfiguration_e = currRightArmConfig


    def printRobotJointInfo(self):
        ### This printing function is for debugging purpose (not used in experiment)
        ############################### information related to Motoman arm (joint info) ###################################
        print "Motoman Robot: " + str(self.motomanGEO_p)
        num_joints = p.getNumJoints(self.motomanGEO_p, self.planningServer)
        print "Num of joints: " + str(num_joints)
        for i in range(num_joints):
          print(p.getJointInfo(self.motomanGEO_p, i, self.planningServer))



