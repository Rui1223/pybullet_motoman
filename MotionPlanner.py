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
    def __init__(self, server, scene_index, camera):
        self.planningServer = server
        self.scene_index = scene_index
        self.camera = camera
        self.taskFolder = self.createtaskFolder(scene_index)
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


    def loadRoadmap(self, handType):
        roadmapFile = os.getcwd() + "/roadmaps/samples_" + handType + ".txt"
        f_roadmap = open(roadmapFile, "r")
        for line in f_roadmap:
            line = line.split()
            line = [float(e) for e in line[1:]]
            self.nodes[handType].append(line)


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
            robot.setSingleArmToConfig(intermNode, handType)
            ### check collision
            if self.collisionAgent_p.collisionCheck_selfCollision(robot.motomanGEO_p) == True:
                # print("self collision")
                return isEdgeValid
            if self.collisionAgent_p.collisionCheck_knownGEO(
                robot.motomanGEO_p, workspace.known_geometries_planning) == True:
                # print("collide with table!")
                return isEdgeValid

        ### Reach here because the edge pass the collision check
        isEdgeValid = True
        return isEdgeValid


    def checkIK(self, ikSolution, ee_idx, desired_ee_pose, robot, workspace):
        robot.setDualArmToConfig(ikSolution)
        isValid = False
        ### first check if IK succeeds
        ### if the ee_idx is within 2.5cm(0.025m) Euclidean distance from the desired one, we accept it
        actual_ee_pose = p.getLinkState(robot.motomanGEO_p, ee_idx)[0]
        ee_dist = self.computePoseDist(actual_ee_pose, desired_ee_pose)
        if ee_dist > 0.02:
            print("Not reachable as expected")
            # raw_input("enter to continue")
            return isValid
        ### Then check if there is collision
        if self.collisionAgent_p.collisionCheck_selfCollision(robot.motomanGEO_p) == True:
            print("self collision!")
            # raw_input("enter to continue")
            return isValid
        if self.collisionAgent_p.collisionCheck_knownGEO(
            robot.motomanGEO_p, workspace.known_geometries_planning) == True:
            print("collision with geometries in the workspace!")
            # raw_input("enter to continue")
            return isValid

        ### If you reach here, the pose pass both IK success and collision check
        isValid = True
        print("it is a valid IK")
        # raw_input("this IK is valid")
        return isValid


    def checkIK_onlyCollision(self, ikSolution, ee_idx, robot, workspace, handType):
        robot.setSingleArmToConfig(ikSolution, handType)
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



    def shortestPathPlanning(self, initialArmConfig, targetArmConfig, theme, robot, workspace, handType):
        ### first prepare the start_goal file
        f = self.writeStartGoal(initialArmConfig, targetArmConfig, theme)
        self.connectStartGoalToArmRoadmap(f, initialArmConfig, targetArmConfig, robot, workspace, handType)

        ### call the planning algorithm
        executeCommand = "./main_planner" + " " + str(self.scene_index) + " " + \
                        theme + " " + handType + " " + str(self.nsamples) + " shortestPath"
        subprocess.call(executeCommand, shell=True)
        ### Now read in trajectory
        traj = self.readTrajectory(theme)
        return traj


    def readTrajectory(self, theme):
        traj = []
        traj_file = self.taskFolder + "/" + theme + "_traj.txt"
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


    def connectStartGoalToArmRoadmap(self, f, initialArmConfig, targetArmConfig, robot, workspace, handType):
        startConnectSuccess = False
        start_id = self.nsamples
        target_id = self.nsamples + 1

        ############# connect the initialArmConfig to the roadmap ###################
        self.nodes[handType].append(initialArmConfig)
        tree = spatial.KDTree(self.nodes[handType])
        queryNode = self.nodes[handType][-1]
        knn = tree.query(queryNode, k=self.nsamples, p=2)
        neighbors_connected = 0
        ### for each potential neighbor
        for j in range(len(knn[1])):
            ### first check if this query node has already connected to enough neighbors
            if neighbors_connected >= self.num_neighbors: 
                break
            if knn[1][j] == len(self.nodes[handType])-1:
                ### if the neighbor is the query node itself
                continue
            ### otherwise, check the edge validity
            neighbor = self.nodes[handType][knn[1][j]]
            isEdgeValid = self.checkEdgeValidity(queryNode, neighbor, robot, workspace, handType)
            if isEdgeValid:
                f.write(str(start_id) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f'))
                f.write("\n")
                neighbors_connected += 1
        print("Number of neighbors for start node " + str(start_id) + ": " + str(neighbors_connected))

        ### at the end, don't forget to delete initialArmConfig from the nodes
        self.nodes[handType].remove(self.nodes[handType][-1])
        ##############################################################################

        ############# connect the targetArmConfig to the roadmap ###################
        self.nodes[handType].append(targetArmConfig)
        tree = spatial.KDTree(self.nodes[handType])
        queryNode = self.nodes[handType][-1]
        knn = tree.query(queryNode, k=self.nsamples, p=2)
        neighbors_connected = 0
        ### for each potential neighbor
        for j in range(len(knn[1])):
            ### first check if this query node has already connected to enough neighbors
            if neighbors_connected >= self.num_neighbors: 
                break
            if knn[1][j] == len(self.nodes[handType])-1:
                ### if the neighbor is the query node itself
                continue
            ### otherwise, check the edge validity
            neighbor = self.nodes[handType][knn[1][j]]
            isEdgeValid = self.checkEdgeValidity(queryNode, neighbor, robot, workspace, handType)
            if isEdgeValid:
                f.write(str(target_id) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f'))
                f.write("\n")
                neighbors_connected += 1
        print("Number of neighbors for goal node " + str(target_id) + ": " + str(neighbors_connected))

        ### at the end, don't forget to delete targetArmConfig from the nodes
        self.nodes[handType].remove(self.nodes[handType][-1])
        ##############################################################################

        robot.resetConfig(robot.homeConfiguration)
        f.close()


    def writeStartGoal(self, initialArmConfig, targetArmConfig, theme):
        start_goal_file = self.taskFolder + "/" + theme + ".txt"
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


    def selectEndEffector(self, handType, robot):
        if handType == "Left":
            ee_idx = robot.left_ee_idx
        else:
            ee_idx = robot.right_ee_idx

        return ee_idx


    def createtaskFolder(self, scene_index):
        ### create a folder to store the roadmap (samples + connections) for the current scene
        taskFolder = os.getcwd() + "/roadmaps/" + scene_index

        if os.path.exists(taskFolder):
            shutil.rmtree(taskFolder)
        os.mkdir(taskFolder)

        return taskFolder


