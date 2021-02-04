#!/usr/bin/env python
from __future__ import division

import pybullet as p
import pybullet_data

import time
import math
from collections import OrderedDict
import IPython

### This file defines the motoman robot of type sda10f ###

class MotomanRobot(object):

    def __init__(self, 
        urdf_filepath, 
        basePosition, baseOrientation, leftArmHomeConfiguration, rightArmHomeConfiguration,
        isPhysicsTurnOn, server):
        ### get the server
        self.server = server
        ### get the urdf file
        self.urdf_filepath = urdf_filepath

        ### reset the base of motoman
        self.basePosition = basePosition
        self.baseOrientation = baseOrientation

        ### load the urdf of the robot
        if isPhysicsTurnOn == False:
            self.motomanGEO = p.loadURDF(
                    fileName=self.urdf_filepath, 
                    basePosition=self.basePosition, baseOrientation=self.baseOrientation, 
                    useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION, physicsClientId=self.server)
        else:
            self.motomanGEO = p.loadURDF(
                    fileName=self.urdf_filepath, 
                    basePosition=self.basePosition, baseOrientation=self.baseOrientation, 
                    useFixedBase=True, physicsClientId=self.server)            
        self.known_geometries = []
        self.known_geometries.append(self.motomanGEO)

        ### set the robot to the home configuration (both left and right arm)
        self.leftArmHomeConfiguration = leftArmHomeConfiguration
        self.rightArmHomeConfiguration = rightArmHomeConfiguration
        self.homeConfiguration = self.leftArmHomeConfiguration + self.rightArmHomeConfiguration

        ################### intrinsic value of the motoman_sda10f ###################
        ### joint and end effector information
        ### end-effector index
        self.left_ee_idx = 10 ### left hand ee
        self.right_ee_idx = 20 ### right hand ee
        ### There is a torso joint which connects the lower and upper body (-2.957 ~ 2.957)
        ### But so far we decide to make that torso joint fixed
        ### For each arm, there are 10 joints and 7 of them are revolute joints
        ### There are total 14 revolute joints for each arm
        ### lower limits for null space
        # self.ll = [-3.13, -1.90, -2.95, -2.36, -3.13, -1.90, -3.13, -3.13, -1.90, -2.95, -2.36, -3.13, -1.90, -3.13]
        # ### upper limits for null space
        # self.ul = [3.13, 1.90, 2.95, 2.36, 3.13, 1.90, 3.13, 3.13, 1.90, 2.95, 2.36, 3.13, 1.90, 3.13]
        # ### joint ranges for null space
        # self.jr = [6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26, 6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26]
        # ### restposes for null space
        # self.rp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        ################## new joint limits ##################
        self.ll = [-3.13, -1.90, -2.95, -1.80, -3.13, -1.90, -3.13, -3.13, -1.90, -2.95, -1.80, -3.13, -1.90, -3.13]
        ### upper limits for null space
        self.ul = [3.13, 1.90, 2.95, 2.36, 3.13, 1.90, 3.13, 3.13, 1.90, 2.95, 2.36, 3.13, 1.90, 3.13]
        ### joint ranges for null space
        self.jr = [6.26, 3.80, 5.90, 4.16, 6.26, 3.80, 6.26, 6.26, 3.80, 5.90, 4.16, 6.26, 3.80, 6.26]
        ### restposes for null space
        self.rp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        ### get all controllable joints (for motoman, they are all revolute joints)
        self.getRobotJointInfo()
        # self.printRJointNames()

        ### initialize the robot's configuration at its home configuration
        self.updateSingleArmConfig(self.leftArmHomeConfiguration, "Left")
        self.updateSingleArmConfig(self.rightArmHomeConfiguration, "Right")
        self.resetArmConfig(self.leftArmCurrConfiguration + self.rightArmCurrConfiguration)


    def updateSingleArmConfig(self, currSingleArmConfig, armType):
        if armType == "Left":
            self.leftArmCurrConfiguration = currSingleArmConfig
        else:
            self.rightArmCurrConfiguration = currSingleArmConfig


    def resetArmConfig(self, resetConfiguration):
        for j in range(1, 8):
            p.resetJointState(self.motomanGEO, j, resetConfiguration[j-1], physicsClientId=self.server)
        for j in range(11, 18):
            p.resetJointState(self.motomanGEO, j, resetConfiguration[j-4], physicsClientId=self.server)

        p.stepSimulation(physicsClientId=self.server)
        left_ee_pos_quat = p.getLinkState(self.motomanGEO, self.left_ee_idx, physicsClientId=self.server)
        self.left_ee_pose = [list(left_ee_pos_quat[0]), list(left_ee_pos_quat[1])]
        right_ee_pos_quat = p.getLinkState(self.motomanGEO, self.right_ee_idx, physicsClientId=self.server)
        self.right_ee_pose = [list(right_ee_pos_quat[0]), list(right_ee_pos_quat[1])]


    def setSingleArmToConfig(self, singleArmConfig, armType):
        ### the planning version of moveSingArm
        if armType == "Left":
            for j in range(1, 8):
                p.resetJointState(self.motomanGEO, j, singleArmConfig[j-1], physicsClientId=self.server)
            for j in range(11, 18):
                p.resetJointState(self.motomanGEO, j, self.rightArmCurrConfiguration[j-11], physicsClientId=self.server)
        else:
            for j in range(1, 8):
                p.resetJointState(self.motomanGEO, j, self.leftArmCurrConfiguration[j-1], physicsClientId=self.server)
            for j in range(11, 18):
                p.resetJointState(self.motomanGEO, j, singleArmConfig[j-11], physicsClientId=self.server)

        # p.stepSimulation(physicsClientId=self.server)
        left_ee_pos_quat = p.getLinkState(self.motomanGEO, self.left_ee_idx, physicsClientId=self.server)
        self.left_ee_pose = [list(left_ee_pos_quat[0]), list(left_ee_pos_quat[1])]
        right_ee_pos_quat = p.getLinkState(self.motomanGEO, self.right_ee_idx, physicsClientId=self.server)
        self.right_ee_pose = [list(right_ee_pos_quat[0]), list(right_ee_pos_quat[1])]


    def moveSingleArm(self, singleArmConfiguration, armType):
        ### move single arm by resetJointState() function
        if armType == "Left":
            for j in range(1, 8):
                p.resetJointState(self.motomanGEO, j, singleArmConfiguration[j-1], physicsClientId=self.server)
            self.updateSingleArmConfig(singleArmConfiguration, armType)

        else:
            for j in range(11, 18):
                p.resetJointState(self.motomanGEO, j, singleArmConfiguration[j-11], physicsClientId=self.server)
            self.updateSingleArmConfig(singleArmConfiguration, armType)

        ###### This function is in maintenance ###### 
        # p.stepSimulation(physicsClientId=self.server)
        left_ee_pos_quat = p.getLinkState(self.motomanGEO, self.left_ee_idx, physicsClientId=self.server)
        self.left_ee_pose = [list(left_ee_pos_quat[0]), list(left_ee_pos_quat[1])]
        right_ee_pos_quat = p.getLinkState(self.motomanGEO, self.right_ee_idx, physicsClientId=self.server)
        self.right_ee_pose = [list(right_ee_pos_quat[0]), list(right_ee_pos_quat[1])]


    def getJointState(self):
        left_joint_state = p.getJointStates(
                bodyUniqueId=self.motomanGEO, jointIndices=range(1, 8), physicsClientId=self.server)
        right_joint_state = p.getJointStates(
                bodyUniqueId=self.motomanGEO, jointIndices=range(11, 18), physicsClientId=self.server)
        temp_left_arm_config = []
        temp_right_arm_config = []
        for joint in left_joint_state:
            temp_left_arm_config.append(joint[0])
        for joint in right_joint_state:
            temp_right_arm_config.append(joint[0])

        self.updateSingleArmConfig(temp_left_arm_config, "Left")
        self.updateSingleArmConfig(temp_right_arm_config, "Right")

        return self.motomanRJointNames, self.leftArmCurrConfiguration + self.rightArmCurrConfiguration


    def getRobotJointInfo(self):
        ################# information related to Motoman arm (joint info) #################
        ### output: self.motomanRJointNames [7 left joints, 7 right joints] 14*1        
        self.motomanRJointNames = []
        num_joints = p.getNumJoints(self.motomanGEO, self.server)
        for i in range(num_joints):
            jointInfo = p.getJointInfo(self.motomanGEO, i, self.server)
            # print(jointInfo)
            if jointInfo[2] == 0:
                ### only get revolute joint
                self.motomanRJointNames.append(jointInfo[1])


    def printRJointNames(self):
        for name in self.motomanRJointNames:
            print(name)




