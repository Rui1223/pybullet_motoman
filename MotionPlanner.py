from __future__ import division
import pybullet as p
import pybullet_data

import os
import random
import shutil


class MotionPlanner(object):
    def __init__(self, nsamples, server, scene_index):
        self.nsamples = nsamples
        self.planningServer = server
        self.scene_index = scene_index

    def PRMstar_generation(self, workspace, robot, handType):
        ### based on the workspace and the robot, let's specified the sampling boundaries
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
        if handType == "Left":
            ee_idx = robot.left_ee_idx
        else:
            ee_idx = robot.right_ee_idx

        self.roadmapFolder, self.samplesFile, self.connectionsFile = self.createRoadmapFolder()
        self.samplingNodes(x_ll, x_ul, y_ll, y_ul, z_ll, z_ul, ee_idx, robot, workspace, handType)


    def samplingNodes(self, x_ll, x_ul, y_ll, y_ul, z_ll, z_ul, ee_idx, robot, workspce, handType):
        # f_samples = open(self.samplesFile, "w")
        nodes = []
        temp_counter = 0

        ### Let's start
        while temp_counter < self.nsamples:
            ### sample a cartesian ee pose and calculate the IK solution
            temp_x = float(format(random.uniform(x_ll, x_ul), '.2f'))
            temp_y = float(format(random.uniform(y_ll, y_ul), '.2f'))
            temp_z = float(format(random.uniform(z_ll, z_ul), '.2f'))
            # ikSolution1 = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO_p, endEffectorLinkIndex=ee_idx,
            #                                     targetPosition=[temp_x, temp_y, temp_z], 
            #                                     lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
            #                                     physicsClientId=self.planningServer)
            ikSolution1 = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO_p, endEffectorLinkIndex=ee_idx,
                                                targetPosition=[temp_x, temp_y, temp_z], 
                                                lowerLimits=robot.ll, upperLimits=robot.ul, 
                                                physicsClientId=self.planningServer)
            print("ikSolution1: " + str(temp_counter))
            if handType == "Left":
                print(ikSolution1[7:14])
            else:
                print(ikSolution1[0:7])
            if handType == "Left":
                for j in range(1, 8):
                    result_p = p.resetJointState(robot.motomanGEO_p, j, ikSolution1[j-1], physicsClientId=self.planningServer)
                # for j in range(11, 18):
                #     result_p = p.resetJointState(robotGEO, j, arm_configuration[j-4], physicsClientId=clientID)
            else:
                # for j in range(1, 8):
                #     result_p = p.resetJointState(robotGEO, j, arm_configuration[j-1], physicsClientId=clientID)
                for j in range(11, 18):
                    result_p = p.resetJointState(robot.motomanGEO_p, j, ikSolution1[j-4], physicsClientId=self.planningServer)
            p.stepSimulation(self.planningServer)
            # self.moveTheArm(ikSolution1, robot.motomanGEO_p, self.planningServer, handType)
            temp_counter += 1



    def moveTheArm(self, arm_configuration, robotGEO, clientID, handType):
        print("arm_configuration: ")
        print(arm_configuration)
        if handType == "Left":
            for j in range(1, 8):
                result_p = p.resetJointState(robotGEO, j, arm_configuration[j-1], physicsClientId=clientID)
            # for j in range(11, 18):
            #     result_p = p.resetJointState(robotGEO, j, arm_configuration[j-4], physicsClientId=clientID)
        else:
            # for j in range(1, 8):
            #     result_p = p.resetJointState(robotGEO, j, arm_configuration[j-1], physicsClientId=clientID)
            for j in range(11, 18):
                result_p = p.resetJointState(robotGEO, j, arm_configuration[j-4], physicsClientId=clientID)
        p.stepSimulation(clientID)


    def createRoadmapFolder(self):
        ### create a folder to store the roadmap (samples + connections) for the current scene
        roadmap_path = os.getcwd() + "/roadmaps/" + self.scene_index

        if os.path.exists(roadmap_path):
            shutil.rmtree(roadmap_path)
        os.mkdir(roadmap_path)

        samplesFile = roadmap_path + "/samples.txt"
        connectionsFile = roadmap_path + "/connections.txt"

        return roadmap_path, samplesFile, connectionsFile
