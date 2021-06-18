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
import IPython

from PybulletPlanScene import PybulletPlanScene
from CollisionChecker import CollisionChecker

import rospy
import rospkg


class RoadmapGenerator(object):


    def generateRoadmap(self, handType):
        self.samplesFile, self.connectionsFile = self.getRoadmapTxt(handType)
        self.samplingNodes(ee_idx, handType)
        self.saveSamplesToFile(self.samplesFile, handType)
        self.roadmapConnect(self.connectionsFile, handType)


    def roadmapConnect(self, connectionsFile, handType):
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



def main(args):
    nsamples = int(args[1])

    ### generate a plan scene
    pybullet_plan_scene = PybulletPlanScene(args)
    # pybullet_plan_scene.planner_p.generateSamples(
    #         nsamples, pybullet_plan_scene.robot_p, pybullet_plan_scene.workspace_p)
    pybullet_plan_scene.planner_p.loadSamples()
    pybullet_plan_scene.planner_p.samplesConnect_cartesian(
                pybullet_plan_scene.robot_p, pybullet_plan_scene.workspace_p, "Left")
    pybullet_plan_scene.planner_p.samplesConnect_cartesian(
                pybullet_plan_scene.robot_p, pybullet_plan_scene.workspace_p, "Right")




if __name__ == '__main__':
    main(sys.argv)
    # roadmap_generator.generateRoadmap("Left")
    # roadmap_generator.generateRoadmap("Right")

