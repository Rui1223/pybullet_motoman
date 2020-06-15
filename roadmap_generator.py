from __future__ import division
import pybullet as p
import pybullet_data

import time
import sys
import os
import random
import numpy as np
from scipy import spatial
import math

from MotomanRobot import MotomanRobot
from Workspace import Workspace
from CollisionChecker import CollisionChecker

class RoadmapGenerator(object):
    def __init__(self, nsamples, robotName, scene):
        self.nsamples = nsamples
        ### get two servers, planning and execution respectively
        self.planningServer = p.connect(p.DIRECT)
        self.executingServer = p.connect(p.DIRECT)
        self.server = [self.planningServer, self.executingServer]
        ### set the robot ready
        if robotName == "Motoman":
            self.robot = MotomanRobot(self.server)
        ### set the workspace (table, shelf, or whatever specified)
        if scene == "Table":
            self.workspace = Workspace("Table", self.robot.BasePosition, self.server)
        ### get all the geometries from the robot and the workspace
        self.planningGeometries = self.robot.known_geometries_planning + self.workspace.known_geometries_planning
        self.executingGeometries = self.robot.known_geometries_executing + self.workspace.known_geometries_executing
        
        self.roadmapFolder = os.getcwd() + "/roadmaps"
        self.nodes = {}
        self.nodes["Left"] = []
        self.nodes["Right"] = []

        self.collisionAgent = CollisionChecker(self.planningServer)


    def generateRoadmap(self, handType):
        self.samplesFile, self.connectionsFile = self.getRoadmapTxt(handType)
        ee_idx = self.selectEndEffector(handType)
        self.samplingNodes(ee_idx, handType)
        self.saveSamplesToFile(self.samplesFile, handType)
        self.roadmapConnect(self.connectionsFile, ee_idx, handType)
      


    def roadmapConnect(self, connectionsFile, ee_idx, handType):
        nsamples = len(self.nodes[handType])
        print("nsamples: " + str(nsamples))
        connectivity = np.zeros((nsamples, nsamples))
        tree = spatial.KDTree(self.nodes[handType]) ### use KD tree to arrange neighbors assignment
        ### first find the parameter k_n used in PRM*
        neighbors_const = 2.5 * math.e * (1 + 1/len(self.nodes[handType][0]))
        ### use k_n to decide the number of neighbors: #neighbors = k_n * log(#samples)
        self.num_neighbors = int(neighbors_const * math.log(nsamples))
        if self.num_neighbors >= nsamples:
            self.num_neighbors = nsamples - 1
        print "num_neighbors: " + str(self.num_neighbors)
        f_roadmap = open(connectionsFile, "w")
        ### for each node
        for i in range(len(self.nodes[handType])):
            queryNode = self.nodes[handType][i]
            knn = tree.query(queryNode, k=nsamples-1, p=2)
            
            neighbors_connected = 0
            ### for each potential neighbor
            for j in range(len(knn[1])):
                ### first check if this query node has already connected to enough neighbors
                if neighbors_connected >= self.num_neighbors: 
                    break
                if knn[1][j] == i:
                    ### if the neighbor is the query node itself
                    continue
                if connectivity[i][knn[1][j]] == 1:
                    ### the connectivity has been checked before
                    neighbors_connected += 1
                    continue
                ### Otherwise, check the edge validity
                ### in terms of collision with the robot itself and all known geometries (e.g. table/shelf)
                ### between the query node and the current neighbor
                neighbor = self.nodes[handType][knn[1][j]]
                isEdgeValid = self.checkEdgeValidity(queryNode, neighbor, handType)
                if isEdgeValid:
                    ### write this edge information with their costs and labels into the txt file
                    f_roadmap.write(str(i) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f') + "\n")
                    connectivity[i][knn[1][j]] = 1
                    connectivity[knn[1][j]][i] = 1
                    neighbors_connected += 1
            print("Number of neighbors for current node " + str(i) + ": " + str(neighbors_connected))

        f_roadmap.close()




    def samplingNodes(self, ee_idx, handType):
        numJoints = int(len(self.robot.homeConfiguration) / 2)

        temp_counter = 0
        ### Let's start
        while temp_counter < self.nsamples:
            ### sample an IK configuration
            ikSolution = self.singleSampling_CSpace(numJoints)
            ### check if the IK solution is valid in terms of
            ### no collision with the robot and other known geometries like table/shelf/etc..
            isValid = self.checkIK_onlyCollision(ikSolution, ee_idx, handType)
            if isValid:
                # print(str(temp_counter) + " " + str(ikSolution))
                self.nodes[handType].append(ikSolution)
                temp_counter += 1
                # raw_input("add this sample. ENTER to continue")



    def singleSampling_CSpace(self, numJoints):
        ### For motoman, the joint limit for the left arm and the right arm is the same
        ikSolution = []
        for i in range(numJoints):
            ikSolution.append(float(format(random.uniform(self.robot.ll[i], self.robot.ul[i]), '.2f')))

        return ikSolution


    def checkEdgeValidity(self, n1, n2, handType):
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
            self.robot.setSingleArmToConfig(intermNode, handType)
            ### check collision
            if self.collisionAgent.collisionCheck_selfCollision(self.robot.motomanGEO_p) == True:
                return isEdgeValid
            if self.collisionAgent.collisionCheck_knownGEO(
                self.robot.motomanGEO_p, self.workspace.known_geometries_planning) == True:
                return isEdgeValid
        ### Reach here because the edge pass the collision check
        isEdgeValid = True
        return isEdgeValid


    def checkIK_onlyCollision(self, ikSolution, ee_idx, handType):
        self.robot.setSingleArmToConfig(ikSolution, handType)
        isValid = False
        ### Then check if there is collision
        if self.collisionAgent.collisionCheck_selfCollision(self.robot.motomanGEO_p) == True:
            # print("self collision!")
            return isValid
        if self.collisionAgent.collisionCheck_knownGEO(
            self.robot.motomanGEO_p, self.workspace.known_geometries_planning) == True:
            # print("collision with geometries in the workspace!")
            return isValid
        ### If you reach here, the sampled config pass both IK success and collision check
        isValid = True
        return isValid


    def saveSamplesToFile(self, samplesFile, handType):
        f_samples = open(samplesFile, "w")
        for i in range(len(self.nodes[handType])):
            node = self.nodes[handType][i]
            f_samples.write(str(i))
            for k in range(len(node)):
                f_samples.write(" " + str(node[k]))
            f_samples.write("\n")
        f_samples.close()


    def selectEndEffector(self, handType):
        if handType == "Left":
            ee_idx = self.robot.left_ee_idx
        else:
            ee_idx = self.robot.right_ee_idx

        return ee_idx



    def getRoadmapTxt(self, handType):
        samplesFile = self.roadmapFolder + "/samples_" + str(handType) + ".txt"
        connectionsFile = self.roadmapFolder + "/connections_" + str(handType) + ".txt"

        return samplesFile, connectionsFile

if __name__ == '__main__':
    nsamples = int(sys.argv[1])
    robotName = sys.argv[2]
    scene = sys.argv[3]
    roadmap_generator = RoadmapGenerator(nsamples, robotName, scene)
    roadmap_generator.generateRoadmap("Left")
    roadmap_generator.generateRoadmap("Right")
    raw_input("Enter to quit.")

