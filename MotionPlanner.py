from __future__ import division
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

from CollisionChecker import CollisionChecker


class MotionPlanner(object):
    def __init__(self, server, scene_index):
        self.planningServer = server
        self.scene_index = scene_index
        self.roadmapFolder = self.createRoadmapFolder(scene_index)
        self.collisionAgent_p = CollisionChecker(self.planningServer)


    def PRMstar_generation(self, workspace, robot, handType, nsamples):
        self.samplesFile, self.connectionsFile = self.getRoadmapTxt(handType)
        self.nsamples = nsamples
        ### based on the workspace and the robot, let's specified the sampling boundaries
        samplingBounds = self.getSamplingBoundary(workspace, robot, handType)
        ### get the ee_idx on which you will perform IK
        ee_idx = self.selectEndEffector(handType, robot)
        ### sampling procedure
        if handType == "Left":
            self.leftNodes = self.samplingNodes(samplingBounds, ee_idx, robot, workspace, handType)
            self.saveSamplesToFile(self.leftNodes, self.samplesFile)
            ### neighbor connection procedure
            self.roadmapConnect(self.leftNodes, ee_idx, workspace, robot, handType)
        else:
            self.rightNodes = self.samplingNodes(samplingBounds, ee_idx, robot, workspace, handType)
            self.saveSamplesToFile(self.rightNodes, self.samplesFile)
            ### neighbor connection procedure
            self.roadmapConnect(self.rightNodes, ee_idx, workspace, robot, handType)
        ### after roadmap regeneration, reset the arm to its previous configuration
        robot.resetConfiguration(robot.homeConfiguration, robot.motomanGEO_p, self.planningServer)


    def roadmapConnect(self, nodes, ee_idx, workspace, robot, handType):
        nsamples = len(nodes)
        connectivity = np.zeros((nsamples, nsamples))
        tree = spatial.KDTree(nodes) ### use KD tree to arrange neighbors assignment
        ### first find the parameter k_n used in PRM*
        neighbors_const = 1.5 * math.e * (1 + 1/len(nodes[0]))
        ### use k_n to decide the number of neighbors: #neighbors = k_n * log(#samples)
        self.num_neighbors = int(neighbors_const * math.log(nsamples))
        if self.num_neighbors >= nsamples:
            self.num_neighbors = nsamples - 1
        print "num_neighbors: " + str(self.num_neighbors)
        f_roadmap = open(self.connectionsFile, "w")
        ### for each node
        for i in range(len(nodes)):
            queryNode = nodes[i]
            knn = tree.query(queryNode, k=self.num_neighbors, p=2)
            # print("check current node " + str(i))
            # print(knn[1])
            ### for each potential neighbor
            for j in range(len(knn[1])):
                if knn[1][j] == i or connectivity[i][knn[1][j]] == 1:
                    ### if the neighbor is the query node itself
                    ### or the connectivity has been checked before
                    ### then skip the edge checking procedure
                    continue
                ### Otherwise, check the edge validity
                ### in terms of collision with the robot itself and all known geometries (e.g. table/shelf)
                ### between the query node and the current neighbor
                neighbor = nodes[knn[1][j]]
                isEdgeValid = self.checkEdgeValidity(queryNode, neighbor, robot, workspace, handType)
                if isEdgeValid:
                    ### write this edge information with their costs and labels into the txt file
                    f_roadmap.write(str(i) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f') + "\n")
                    connectivity[i][knn[1][j]] = 1
                    connectivity[knn[1][j]][i] = 1
        f_roadmap.close()


    def samplingNodes(self, samplingBounds, ee_idx, robot, workspace, handType):
        numJoints = int(len(robot.homeConfiguration) / 2)
        ### first figure out whether it is a left hand or right hand
        if handType == "Left":
            j = 0
        else:
            j = 0 + numJoints

        nodes = []
        temp_counter = 0

        ### Let's start
        while temp_counter < self.nsamples:
            ### sample a cartesian ee pose and calculate the IK solution
            if temp_counter < int(self.nsamples*0.01):
                temp_sample = self.singleSampling_CartesianSpace(samplingBounds)
                ikSolution = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO_p, endEffectorLinkIndex=ee_idx,
                                                    targetPosition=temp_sample, 
                                                    lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                                                    physicsClientId=self.planningServer)
                ikSolution = [ float(format(jointAngle, '.2f')) for jointAngle in ikSolution[j:j+numJoints] ]
            else:
                ikSolution = self.singleSampling_CSpace(robot)
            ### check if the IK solution is valid in terms of 
            ### no collision with the robot and other known geometries like table/shelf/etc..
            isValid = self.checkIK_onlyCollision(ikSolution, ee_idx, robot, workspace, handType)
            if isValid:
                # print str(temp_counter) + ": " + str(temp_sample)
                nodes.append(ikSolution)
                temp_counter += 1
                # raw_input("add this sample. ENTER to continue")

        return nodes


    def checkEdgeValidity(self, n1, n2, robot, workspace, handType):
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
            robot.moveSingleArm(intermNode, robot.motomanGEO_p, handType, self.planningServer)
            ### check collision
            if self.collisionAgent_p.collisionCheck_selfCollision(robot.motomanGEO_p) == True:
                return isEdgeValid
            if self.collisionAgent_p.collisionCheck_knownGEO(
                robot.motomanGEO_p, workspace.known_geometries_planning) == True:
                return isEdgeValid

        ### Reach here because the edge pass the collision check
        isEdgeValid = True
        return isEdgeValid


    def checkIK(self, ikSolution, ee_idx, desired_ee_pose, robot, workspace, handType):
        robot.moveSingleArm(ikSolution, robot.motomanGEO_p, handType, self.planningServer)
        isValid = False
        ### first check if IK succeeds
        ### if the ee_idx is within 2.5cm(0.025m) Euclidean distance from the desired one, we accept it
        actual_ee_pose = p.getLinkState(robot.motomanGEO_p, ee_idx)[0]
        ee_dist = self.computePoseDist(actual_ee_pose, desired_ee_pose)
        if ee_dist > 0.02:
            # print("Not reachable as expected")
            return isValid
        ### Then check if there is collision
        if self.collisionAgent_p.collisionCheck_selfCollision(robot.motomanGEO_p) == True:
            # print("self collision!")
            return isValid
        if self.collisionAgent_p.collisionCheck_knownGEO(
            robot.motomanGEO_p, workspace.known_geometries_planning) == True:
            # print("collision with geometries in the workspace!")
            return isValid

        ### If you reach here, the pose pass both IK success and collision check
        isValid = True
        return isValid


    def checkIK_onlyCollision(self, ikSolution, ee_idx, robot, workspace, handType):
        robot.moveSingleArm(ikSolution, robot.motomanGEO_p, handType, self.planningServer)
        isValid = False
        ### Then check if there is collision
        if self.collisionAgent_p.collisionCheck_selfCollision(robot.motomanGEO_p) == True:
            # print("self collision!")
            return isValid
        if self.collisionAgent_p.collisionCheck_knownGEO(
            robot.motomanGEO_p, workspace.known_geometries_planning) == True:
            # print("collision with geometries in the workspace!")
            return isValid

        ### If you reach here, the pose pass both IK success and collision check
        isValid = True
        return isValid


    def computePoseDist(self, actual_pose, desired_pose):
        temp_dist = 0.0
        for i in range(len(actual_pose)):
            temp_dist += (actual_pose[i] - desired_pose[i])**2
        temp_dist = math.sqrt(temp_dist)

        return temp_dist


    def singleSampling_CartesianSpace(self, samplingBounds):
        temp_x = float(format(random.uniform(samplingBounds[0][0], samplingBounds[0][1]), '.2f'))
        temp_y = float(format(random.uniform(samplingBounds[1][0], samplingBounds[1][1]), '.2f'))
        temp_z = float(format(random.uniform(samplingBounds[2][0], samplingBounds[2][1]), '.2f'))

        return [temp_x, temp_y, temp_z]

    def singleSampling_CSpace(self, robot):
        temp_j0 = float(format(random.uniform(robot.ll[0], robot.ul[0]), '.2f'))
        temp_j1 = float(format(random.uniform(robot.ll[1], robot.ul[1]), '.2f'))
        temp_j2 = float(format(random.uniform(robot.ll[2], robot.ul[2]), '.2f'))
        temp_j3 = float(format(random.uniform(robot.ll[3], robot.ul[3]), '.2f'))
        temp_j4 = float(format(random.uniform(robot.ll[4], robot.ul[4]), '.2f'))
        temp_j5 = float(format(random.uniform(robot.ll[5], robot.ul[5]), '.2f'))
        temp_j6 = float(format(random.uniform(robot.ll[6], robot.ul[6]), '.2f'))
        return [temp_j0, temp_j1, temp_j2, temp_j3, temp_j4, temp_j5, temp_j6]


    def getSamplingBoundary(self, workspace, robot, handType):
        if handType == "Left":
            x_ll = robot.BasePosition[0] - 0.35
            x_ul = workspace.tablePosition[0] + workspace.table_dim[0] / 2
            y_ll = robot.BasePosition[1] - 0.05
            y_ul = workspace.tablePosition[1] + workspace.table_dim[1] / 2 + 0.15
            z_ll = workspace.tablePosition[2] + workspace.table_dim[2] / 2
            z_ul = z_ll + 0.7
        else:
            x_ll = robot.BasePosition[0] - 0.35
            x_ul = workspace.tablePosition[0] + workspace.table_dim[0] / 2
            y_ll = workspace.tablePosition[1] - workspace.table_dim[1] / 2 - 0.15
            y_ul = 0.05
            z_ll = workspace.tablePosition[2] + workspace.table_dim[2] / 2
            z_ul = z_ll + 0.7

        return [[x_ll, x_ul], [y_ll, y_ul], [z_ll, z_ul]]

    def shortestPathPlanning(self, initialArmConfig, targetArmConfig, theme, robot, workspace, handType):
        ### first prepare the start_goal file
        f = self.writeStartGoal(initialArmConfig, targetArmConfig, theme)
        if handType == "Left":
            self.connectStartGoalToLeftArmRoadmap(f, initialArmConfig, targetArmConfig, robot, workspace, handType)
        else:
            self.connectStartGoalToRightArmRoadmap(f, initialArmConfig, targetArmConfig, robot, workspace, handType)
        ### call the planning algorithm
        executeCommand = "./main_planner" + " " + str(self.scene_index) + " " + \
                        theme + " " + handType + " " + str(self.nsamples) + " shortestPath"
        subprocess.call(executeCommand, shell=True)
        ### Now read in trajectory
        traj = self.readTrajectory(theme)
        return traj


    def readTrajectory(self, theme):
        traj = []
        traj_file = self.roadmapFolder + "/" + theme + "_traj.txt"
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


    def connectStartGoalToLeftArmRoadmap(self, f, initialArmConfig, targetArmConfig, robot, workspace, handType):
        startConnectSuccess = False
        start_id = self.nsamples
        target_id = self.nsamples + 1

        ############# connect the initialArmConfig to the roadmap ###################
        self.leftNodes.append(initialArmConfig)

        tree = spatial.KDTree(self.leftNodes)
        queryNode = self.leftNodes[-1]
        knn = tree.query(queryNode, k=self.num_neighbors, p=2)
        # print("start: ")
        # print(knn[1])
        ### for each neighbor
        for j in range(len(knn[1])):
            if knn[1][j] == len(self.leftNodes)-1: 
                continue
            else:
                ### check collision
                # print("neighbor " + str(knn[1][j]))
                neighbor = self.leftNodes[knn[1][j]]
                isEdgeValid = self.checkEdgeValidity(queryNode, neighbor, robot, workspace, handType)
                # raw_input("Press to continue")
                if isEdgeValid:
                    f.write(str(start_id) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f'))
                    f.write("\n")

        ### at the end, don't forget to delete initialArmConfig from the nodes
        self.leftNodes.remove(self.leftNodes[-1])
        ##############################################################################

        ############# connect the targetArmConfig to the roadmap ###################
        self.leftNodes.append(targetArmConfig)

        tree = spatial.KDTree(self.leftNodes)
        queryNode = self.leftNodes[-1]
        knn = tree.query(queryNode, k=self.num_neighbors, p=2)
        # print("goal: ")
        # print(knn[1])
        ### for each neighbor
        for j in range(len(knn[1])):
            if knn[1][j] == len(self.leftNodes)-1: 
                continue
            else:
                ### check collision
                # print("neighbor " + str(knn[1][j]))
                neighbor = self.leftNodes[knn[1][j]]
                isEdgeValid = self.checkEdgeValidity(queryNode, neighbor, robot, workspace, handType)
                # raw_input("Press to continue")
                if isEdgeValid:
                    f.write(str(target_id) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f'))
                    f.write("\n")

        ### at the end, don't forget to delete initialArmConfig from the nodes
        self.leftNodes.remove(self.leftNodes[-1])
        ##############################################################################

        robot.resetConfiguration(robot.homeConfiguration, robot.motomanGEO_p, self.planningServer)

        f.close()




    def writeStartGoal(self, initialArmConfig, targetArmConfig, theme):
        start_goal_file = self.roadmapFolder + "/" + theme + ".txt"
        f = open(start_goal_file, "w")
        f.write(str(self.nsamples) + " ")
        for e in initialArmConfig:
            f.write(str(e) + " ")
        f.write("\n")
        f.write(str(self.nsamples+1) + " ")
        for e in targetArmConfig:
            f.write(str(e) + " ")
        f.write("\n")

        return f


    def saveSamplesToFile(self, nodes, samplesFile):
        f_samples = open(samplesFile, "w")
        for i in range(len(nodes)):
            node = nodes[i]
            f_samples.write(str(i))
            for k in range(len(node)):
                f_samples.write(" " + str(node[k]))
            f_samples.write("\n")
        f_samples.close()


    def selectEndEffector(self, handType, robot):
        if handType == "Left":
            ee_idx = robot.left_ee_idx
        else:
            ee_idx = robot.right_ee_idx

        return ee_idx


    def createRoadmapFolder(self, scene_index):
        ### create a folder to store the roadmap (samples + connections) for the current scene
        roadmapFolder = os.getcwd() + "/roadmaps/" + scene_index

        if os.path.exists(roadmapFolder):
            shutil.rmtree(roadmapFolder)
        os.mkdir(roadmapFolder)

        return roadmapFolder


    def getRoadmapTxt(self, handType):

        samplesFile = self.roadmapFolder + "/samples_" + str(handType) + ".txt"
        connectionsFile = self.roadmapFolder + "/connections_" + str(handType) + ".txt"

        return samplesFile, connectionsFile
