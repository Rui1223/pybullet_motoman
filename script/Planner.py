from __future__ import division
from __future__ import print_function
import pybullet as p
import pybullet_data

import os
import random
import shutil
import math
import numpy as np
from scipy import spatial
import time
import IPython
import subprocess

import utils
from CollisionChecker import CollisionChecker

import rospy
from rospkg import RosPack

class Planner(object):
    def __init__(self, server):
        self.planningServer = server
        self.collisionAgent_p = CollisionChecker(self.planningServer)
        self.nodes = {}
        self.nodes["Left"] = []
        self.nodes["Right"] = []
        self.loadRoadmap("Left")
        self.loadRoadmap("Right")
        self.nsamples = len(self.nodes["Left"])
        neighbors_const = 2.5 * math.e * (1 + 1/len(self.nodes["Left"][0]))
        ### use k_n to decide the number of neighbors: #neighbors = k_n * log(#samples)
        self.num_neighbors = int(neighbors_const * math.log(self.nsamples))

    def loadRoadmap(self, armType):
        rospack = RosPack()
        samplesFile = os.path.join(rospack.get_path("pybullet_motoman"), "roadmaps/samples_" + armType + ".txt")
        f_samples = open(samplesFile, "r")
        for line in f_samples:
            line = line.split()
            line = [float(e) for e in line[1:]]
            self.nodes[armType].append(line)


    def generateConfigFromPose(self, pose3D, robot, workspace, armType):
        ### This function convert a pose3D object into a robot arm configuration
        ### Output: a configuration of a single arm (7*1 list)
        pose = [[pose3D.position.x, pose3D.position.y, pose3D.position.z],
                [pose3D.orientation.x, pose3D.orientation.y, pose3D.orientation.z, pose3D.orientation.w]]

        if armType == "Left":
            ee_idx = robot.left_ee_idx
            first_joint_index = 0
        else:
            ee_idx = robot.right_ee_idx
            first_joint_index = 7

        config_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                endEffectorLinkIndex=ee_idx,
                                targetPosition=pose[0],
                                targetOrientation=pose[1],
                                lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                                maxNumIterations=20000, residualThreshold=0.0000001,
                                physicsClientId=robot.server)
        singleArmConfig_IK = config_IK[first_joint_index:first_joint_index+7]
        isValid = self.checkIK(singleArmConfig_IK, ee_idx, pose[0], robot, workspace, armType)

        trials = 0
        while (not isValid) and (trials < 5):
            ### reset arm configuration
            resetSingleArmConfiguration = [0.0]*len(robot.leftArmCurrConfiguration)
            robot.setSingleArmToConfig(resetSingleArmConfiguration, armType)

            config_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                    endEffectorLinkIndex=ee_idx,
                                    targetPosition=pose[0],
                                    targetOrientation=pose[1],
                                    lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                                    maxNumIterations=20000, residualThreshold=0.0000001,
                                    physicsClientId=robot.server)
            singleArmConfig_IK = config_IK[first_joint_index:first_joint_index+7]
            isValid = self.checkIK(singleArmConfig_IK, ee_idx, pose[0], robot, workspace, armType)
            if isValid: break
            ### otherwise
            trials += 1

        print("forget about pre-grasp at this moment. Will come back later")

        return singleArmConfig_IK



    def checkIK(self, singleArmConfig_IK, ee_idx, desired_ee_pose, robot, workspace, armType):
        ### This function checks if an IK solution is valid in terms of
        ### (0) if the joint values exceed the limit
        ### (1) no collision with the robot and the workspace
        ### (2) small error from the desired pose (so far only check position error)

        robot.setSingleArmToConfig(singleArmConfig_IK, armType)
        isValid = False
        ### first check if IK success
        ### if the ee_idx is within 2.0cm(0.02m) Euclidean distance from the desired one, we accept it
        actual_ee_pose = p.getLinkState(robot.motomanGEO, ee_idx, physicsClientId=robot.server)[0]
        ee_dist = utils.computePoseDist(actual_ee_pose, desired_ee_pose)
        if ee_dist > 0.02:
            print("Not reachable as expected")
            return isValid
        ### Then check if there is collision
        if self.collisionAgent_p.collisionCheck_selfCollision(robot.motomanGEO) == True:
            print("self collision!")
            return isValid
        if self.collisionAgent_p.collisionCheck_knownGEO(
                    robot.motomanGEO, workspace.known_geometries) == True:
            print("collision with known GEO")
            return isValid

        ### If you reach here, the pose passes both IK success and collision check
        isValid = True
        print("it is a valid IK")
        return isValid


    def checkEdgeValidity(self, n1, n2, robot, workspace, armType):
        nseg = 5

        isEdgeValid = False
        for i in range(1, nseg):
            interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
            interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
            interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
            interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
            interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
            interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
            interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
            intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
            robot.setSingleArmToConfig(intermNode, armType)
            ### check collision
            if self.collisionAgent_p.collisionCheck_selfCollision(robot.motomanGEO) == True:
                # print("self collision")
                return isEdgeValid
            if self.collisionAgent_p.collisionCheck_knownGEO(
                robot.motomanGEO, workspace.known_geometries) == True:
                # print("collide with table!")
                return isEdgeValid

        ### Reach here because the edge pass the collision check
        isEdgeValid = True
        return isEdgeValid


    def shortestPathPlanning(self,
            initialSingleArmConfig, targetSingleArmConfig,
            theme, robot, workspace, armType):
        ### The output is a traj ( a list of waypoints (7*1 list))
        ### first prepare the start_goal file
        f = self.writeStartGoal(initialSingleArmConfig, targetSingleArmConfig, theme)
        self.connectStartGoalToArmRoadmap(f, initialSingleArmConfig, targetSingleArmConfig, robot, workspace, armType)
        ### move back the robot arm back to its current configuration after collision check
        robot.resetArmConfig(robot.leftArmCurrConfiguration + robot.rightArmCurrConfiguration)

        ### call the planning algorithm
        executeCommand = "./main_planner" + " " + theme + " " + armType + " " + str(self.nsamples) + " shortestPath"
        # subprocess.call(executeCommand, cwd="/home/rui/Documents/research/motoman_ws/src/pybullet_motoman/src", shell=True)
        rospack = RosPack()
        cwd = os.path.join(rospack.get_path("pybullet_motoman"), 'src')
        subprocess.call(executeCommand, cwd=cwd, shell=True)
        ### Now read in the trajectory
        traj = self.readTrajectory(theme)

        return traj


    def readTrajectory(self, theme):
        traj = []
        # traj_file = "/home/rui/Documents/research/motoman_ws/src/pybullet_motoman/" + theme + "_traj.txt"
        rospack = RosPack()
        traj_file = os.path.join(rospack.get_path("pybullet_motoman"), theme + '_traj.txt')
        f = open(traj_file, "r")
        nline = 0
        for line in f:
            nline += 1
            if nline == 1:
                line = line.split()
                isFailure = bool(int(line[0]))
                if isFailure:
                    f.close()
                    return traj
            else:
                line = line.split()
                line = [float(e) for e in line]
                traj.append(line)
        f.close()

        return traj


    def writeStartGoal(self, initialSingleArmConfig, targetSingleArmConfig, theme):
        rospack = RosPack()
        start_goal_file = os.path.join(rospack.get_path("pybullet_motoman"), theme + '.txt')
        # start_goal_file = "/home/rui/Documents/research/motoman_ws/src/pybullet_motoman/" + theme + ".txt"
        f = open(start_goal_file, "w")
        f.write(str(self.nsamples) + " ")
        for e in initialSingleArmConfig:
            f.write(str(e) + " ")
        f.write("\n")
        f.write(str(self.nsamples+1) + " ")
        for e in targetSingleArmConfig:
            f.write(str(e) + " ")
        f.write("\n")

        return f


    def connectStartGoalToArmRoadmap(self,
        f, initialSingleArmConfig, targetSingleArmConfig, robot, workspace, armType):
        startConnectSuccess = False
        start_id = self.nsamples
        target_id = self.nsamples + 1

        ############# connect the initialSingleArmConfig to the roadmap ###################
        self.nodes[armType].append(initialSingleArmConfig)
        tree = spatial.KDTree(self.nodes[armType])
        queryNode = self.nodes[armType][-1]
        knn = tree.query(queryNode, k=self.nsamples, p=2)
        neighbors_connected = 0
        ### for each potential neighbor
        for j in range(len(knn[1])):
            ### first check if this query node has already connected to enough neighbors
            if neighbors_connected >= self.num_neighbors:
                break
            if knn[1][j] == len(self.nodes[armType])-1:
                ### if the neighbor is the query node itself
                continue
            ### otherwise, check the edge validity
            neighbor = self.nodes[armType][knn[1][j]]
            isEdgeValid = self.checkEdgeValidity(queryNode, neighbor, robot, workspace, armType)
            if isEdgeValid:
                f.write(str(start_id) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f'))
                f.write("\n")
                neighbors_connected += 1
        print("Number of neighbors for start node " + str(start_id) + ": " + str(neighbors_connected))

        ### at the end, don't forget to delete initialSingleArmConfig from the nodes
        self.nodes[armType].remove(self.nodes[armType][-1])
        ##############################################################################

        ############# connect the targetSingleArmConfig to the roadmap ###################
        self.nodes[armType].append(targetSingleArmConfig)
        tree = spatial.KDTree(self.nodes[armType])
        queryNode = self.nodes[armType][-1]
        knn = tree.query(queryNode, k=self.nsamples, p=2)
        neighbors_connected = 0
        ### for each potential neighbor
        for j in range(len(knn[1])):
            ### first check if this query node has already connected to enough neighbors
            if neighbors_connected >= self.num_neighbors:
                break
            if knn[1][j] == len(self.nodes[armType])-1:
                ### if the neighbor is the query node itself
                continue
            ### otherwise, check the edge validity
            neighbor = self.nodes[armType][knn[1][j]]
            isEdgeValid = self.checkEdgeValidity(queryNode, neighbor, robot, workspace, armType)
            if isEdgeValid:
                f.write(str(target_id) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f'))
                f.write("\n")
                neighbors_connected += 1
        print("Number of neighbors for goal node " + str(target_id) + ": " + str(neighbors_connected))

        ### at the end, don't forget to delete targetSingleArmConfig from the nodes
        self.nodes[armType].remove(self.nodes[armType][-1])
        ##############################################################################

        f.close()