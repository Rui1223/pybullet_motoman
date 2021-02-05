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
from operator import itemgetter
# import sklearn
# from sklearn.neighbors import NearestNeighbors

import utils
from CollisionChecker import CollisionChecker

import rospy
from rospkg import RosPack

class Planner(object):
    def __init__(self, rosPackagePath, server,
        isObjectInLeftHand=False, isObjectInRightHand=False,
        objectInLeftHand=None, objectInRightHand=None):
        self.planningServer = server
        self.rosPackagePath = rosPackagePath
        self.roadmapFolder = os.path.join(self.rosPackagePath, "roadmaps")
        self.collisionAgent_p = CollisionChecker(self.planningServer)
        self.nodes = {}
        self.nodes["Left"] = []
        self.nodes["Right"] = []
        self.workspaceNodes = {}
        self.workspaceNodes["Left"] = [] ### ([x,y,z (position),w,x,y,z (quaternion)])
        self.workspaceNodes["Right"] = []

        self.weight_option=0
        ### weight_r is the weight for the distance contributed by quaternion
        self.weight_r = self.weight_option*0.01

        self.isObjectInLeftHand = isObjectInLeftHand
        self.isObjectInRightHand = isObjectInRightHand
        self.objectInLeftHand = objectInLeftHand
        self.objectInRightHand = objectInRightHand
        self.leftLocalPose = [[-1, -1, -1], [-1, -1, -1, -1]]
        self.rightLocalPose = [[-1, -1, -1], [-1, -1, -1, -1]]


    def loadSamples(self):
        arms = ["Left", "Right"]
        for armType in arms:
            samplesFile = self.roadmapFolder + "/samples_" + str(armType) + ".txt"
            samplesWorkspaceFile = self.roadmapFolder + "/samplesWorkspace_" + str(armType) + ".txt"

            f_samples = open(samplesFile, "r")
            for line in f_samples:
                line = line.split()
                line = [float(e) for e in line[1:]]
                self.nodes[armType].append(line)
            f_samples.close()

            f_samplesWorkspace = open(samplesWorkspaceFile, "r")
            for line in f_samplesWorkspace:
                line = line.split()
                line = [float(e) for e in line[1:]]
                self.workspaceNodes[armType].append(line)
            f_samplesWorkspace.close()

        ### specify the needed parameters
        self.nsamples = len(self.nodes["Left"])
        self.neighbors_const = 2.5 * math.e * (1 + 1/len(self.workspaceNodes["Left"][0]))
        ### use k_n to decide the number of neighbors: #neighbors = k_n * log(#samples)
        self.num_neighbors = int(self.neighbors_const * math.log(self.nsamples))
        if self.num_neighbors > self.nsamples:
            self.num_neighbors = self.nsamples
        print("nsamples: ", self.nsamples)
        print("num_neighbors: ", self.num_neighbors)


    def generateSamples(self, nsamples, robot, workspace):
        self.nsamples = nsamples
        arms = ["Left", "Right"]
        for armType in arms:
            samplesFile = self.roadmapFolder + "/samples_" + str(armType) + ".txt"
            samplesWorkspaceFile = self.roadmapFolder + "/samplesWorkspace_" + str(armType) + ".txt"
            if armType == "Left":
                ee_idx = robot.left_ee_idx
            else:
                ee_idx = robot.right_ee_idx
            self.samplingNodes(ee_idx, robot, workspace, armType)
            self.saveSamplesToFile(samplesFile, samplesWorkspaceFile, armType)

        ### specify the needed parameters
        self.neighbors_const = 2.5 * math.e * (1 + 1/len(self.workspaceNodes["Left"][0]))
        ### use k_n to decide the number of neighbors: #neighbors = k_n * log(#samples)
        self.num_neighbors = int(self.neighbors_const * math.log(self.nsamples))
        if self.num_neighbors > self.nsamples:
            self.num_neighbors = self.nsamples
        print("nsamples: ", self.nsamples)
        print("num_neighbors: ", self.num_neighbors)


    def samplingNodes(self, ee_idx, robot, workspace, armType):
        numJoints = int(len(robot.leftArmHomeConfiguration))
        temp_counter = 0
        ### Let's start
        while temp_counter < self.nsamples:
            # if temp_counter % 100 == 0:
            #     print("Now finish " + str(temp_counter) + " samples.")
            ### sample an IK configuration
            ikSolution = self.singleSampling_CSpace(robot) ### this is for a single arm
            ### check if the IK solution is valid in terms of
            ### no collision with the robot and other known geometries like table/shelf/etc..
            isValid = self.checkIK_CollisionWithRobotAndKnownGEO(ikSolution, robot, workspace, armType)
            # print("isValid? " + str(isValid))
            if isValid:
                # print(str(temp_counter) + " " + str(ikSolution))
                state = p.getLinkState(robot.motomanGEO, ee_idx, physicsClientId=robot.server)
                # print(str(temp_counter) + " " + str(list(state[0])))
                self.nodes[armType].append(list(state[0]) + list(state[1]))
                self.workspaceNodes[armType].append(list(state[0]))
                temp_counter += 1


    def samplesConnect_cartesian(self, robot, workspace, armType):
        connectivity = np.zeros((self.nsamples, self.nsamples))
        tree = spatial.KDTree(self.workspaceNodes[armType]) ### use KD tree to arrange neighbors assignment
        connectionsFile = self.roadmapFolder + "/connections_" + str(armType) + ".txt"
        f_connection = open(connectionsFile, "w")
        ### for each node
        for i in range(len(self.workspaceNodes[armType])):
            queryworkspaceNode = self.workspaceNodes[armType][i]
            knn = tree.query(queryworkspaceNode, k=self.num_neighbors, p=2)
            queryNode = self.nodes[armType][i]

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
                neighbor = self.nodes[armType][knn[1][j]]
                # print("query node: ", queryNode)
                # print("neighbor: ", neighbor)
                # raw_input("check")
                isEdgeValid = self.checkEdgeValidity_cartesian(queryNode, neighbor, robot, workspace, armType)
                if isEdgeValid:
                    ### write this edge information with their costs and labels into the txt file
                    f_connection.write(str(i) + " " + str(knn[1][j]) + " " + str(knn[0][j]) + "\n")
                    connectivity[i][knn[1][j]] = 1
                    connectivity[knn[1][j]][i] = 1
                    neighbors_connected += 1
            print("Number of neighbors for current node " + str(i) + ": " + str(neighbors_connected))
        f_connection.close()


    def samplesConnect(self, robot, workspace, armType):
        connectivity = np.zeros((self.nsamples, self.nsamples))
        tree = spatial.KDTree(self.workspaceNodes[armType]) ### use KD tree to arrange neighbors assignment
        connectionsFile = self.roadmapFolder + "/connections_" + str(armType) + ".txt"
        f_connection = open(connectionsFile, "w")
        ### for each node
        for i in range(len(self.workspaceNodes[armType])):
            queryNode = self.workspaceNodes[armType][i]
            query_config = self.nodes[armType][i]
            knn = tree.query(queryNode, k=self.num_neighbors, p=2)

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
                neighbor = self.nodes[armType][knn[1][j]]
                isEdgeValid = self.checkEdgeValidity(query_config, neighbor, robot, workspace, armType)
                if isEdgeValid:
                    ### write this edge information with their costs and labels into the txt file
                    f_connection.write(str(i) + " " + str(knn[1][j]) + " " + str(knn[0][j]) + "\n")
                    connectivity[i][knn[1][j]] = 1
                    connectivity[knn[1][j]][i] = 1
                    neighbors_connected += 1
            print("Number of neighbors for current node " + str(i) + ": " + str(neighbors_connected))
        f_connection.close()


    def singleSampling_CSpace(self, robot):
        ### For motoman, the joint limit for the left arm and the right arm is the same
        numJoints = int(len(robot.leftArmHomeConfiguration))
        ikSolution = []
        for i in range(numJoints):
            ikSolution.append(random.uniform(robot.ll[i], robot.ul[i]))

        return ikSolution


    def saveSamplesToFile(self, samplesFile, samplesWorkspaceFile, armType):
        f_samples = open(samplesFile, "w")
        for i in range(len(self.nodes[armType])):
            node = self.nodes[armType][i]
            f_samples.write(str(i))
            for k in range(len(node)):
                f_samples.write(" " + str(node[k]))
            f_samples.write("\n")
        f_samples.close()

        f_samplesWorkspace = open(samplesWorkspaceFile, "w")
        for i in range(len(self.workspaceNodes[armType])):
            node = self.workspaceNodes[armType][i]
            f_samplesWorkspace.write(str(i))
            for k in range(len(node)):
                f_samplesWorkspace.write(" " + str(node[k]))
            f_samplesWorkspace.write("\n")
        f_samplesWorkspace.close()


    def updateManipulationStatus(self, isObjectInLeftHand, isObjectInRightHand, 
            leftLocalPose, rightLocalPose, objectGEO):
        ### This update update the manipulation status by indicating
        ### whether the object is in any of the hand
        self.isObjectInLeftHand = isObjectInLeftHand
        self.isObjectInRightHand = isObjectInRightHand

        if self.isObjectInLeftHand:
            self.objectInLeftHand = objectGEO
            self.leftLocalPose = leftLocalPose
        else:
            self.objectInLeftHand = None
            self.leftLocalPose = leftLocalPose

        if self.isObjectInRightHand:
            self.objectInRightHand = objectGEO
            self.rightLocalPose = rightLocalPose
        else:
            self.objectInRightHand = None
            self.rightLocalPose = rightLocalPose


    def IKresetForSingleArm(self, robot, armType):
        if armType == "Left":
            first_joint_index = 0
            config_length = len(robot.leftArmCurrConfiguration)
        else:
            first_joint_index = 7
            config_length = len(robot.rightArmCurrConfiguration)

        ### reset arm configuration
        resetSingleArmConfiguration = []
        for i in range(config_length):
            resetSingleArmConfiguration.append(
                    random.uniform(robot.ll[first_joint_index + i], robot.ul[first_joint_index + i]))

        # resetSingleArmConfiguration = [0.0]*len(robot.leftArmCurrConfiguration)
        robot.setSingleArmToConfig(resetSingleArmConfiguration, armType)


    def generatePreGrasp(self, grasp_pose, robot, workspace, armType, motionType):
        ### This function generates a pre-grasp pose based on grasp pose
        ### It is 10cm (0.1m) behind the approaching direction of z axis (local axis of the end effector)
        ### Input: grasp_pose: [[x,y,z], [x,y,z,w]]
        if armType == "Left":
            ee_idx = robot.left_ee_idx
            first_joint_index = 0
        else:
            ee_idx = robot.right_ee_idx
            first_joint_index = 7

        temp_rot_matrix = p.getMatrixFromQuaternion(grasp_pose[1])
        ### local z-axis of the end effector
        temp_approaching_direction = [temp_rot_matrix[2], temp_rot_matrix[5], temp_rot_matrix[8]]
        temp_pos = list(np.array(grasp_pose[0]) - 0.1*np.array(temp_approaching_direction))

        preGrasp_pose = [temp_pos, grasp_pose[1]] ### the quaternion remains the same as grasp_pose
        ### check the IK the pre-grasp pose
        q_preGraspIK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO, 
                                endEffectorLinkIndex=ee_idx, 
                                targetPosition=preGrasp_pose[0], 
                                targetOrientation=preGrasp_pose[1], 
                                lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr, 
                                maxNumIterations=2000, residualThreshold=0.0000001,
                                physicsClientId=robot.server)
        singleArmConfig_IK = q_preGraspIK[first_joint_index:first_joint_index+7]
        isPoseValid = self.checkIK(
                singleArmConfig_IK, ee_idx, preGrasp_pose, robot, workspace, armType, motionType)

        trials = 0
        while (not isPoseValid) and (trials < 5):
            if checkType == "discrete":
                ### reset arm configuration
                self.IKresetForSingleArm(robot, armType)
            ### try another IK
            q_preGraspIK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                    endEffectorLinkIndex=ee_idx,
                                    targetPosition=preGrasp_pose[0],
                                    targetOrientation=preGrasp_pose[1],
                                    lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                                    maxNumIterations=2000, residualThreshold=0.0000001,
                                    physicsClientId=robot.server)
            singleArmConfig_IK = q_preGraspIK[first_joint_index:first_joint_index+7]
            isPoseValid = self.checkIK(
                singleArmConfig_IK, ee_idx, preGrasp_pose, robot, workspace, armType, motionType)
            if isPoseValid: break
            ### otherwise
            trials += 1
        ### you need to return both the statement whether the pose is valid
        ### and the valid configuration the pose corresponds to
        return isPoseValid, preGrasp_pose, singleArmConfig_IK


    def updateRealObjectBasedonLocalPose(self, robot, armType):
        if armType == "Left":
            ee_idx = robot.left_ee_idx
            objectInHand = self.objectInLeftHand
            curr_ee_pose = robot.left_ee_pose
            object_global_pose = self.getObjectGlobalPose(self.leftLocalPose, curr_ee_pose)

        else:
            ee_idx = robot.right_ee_idx
            objectInHand = self.objectInRightHand
            curr_ee_pose = robot.right_ee_idx
            object_global_pose = self.getObjectGlobalPose(self.rightLocalPose, curr_ee_pose)

        p.resetBasePositionAndOrientation(
            objectInHand, object_global_pose[0], object_global_pose[1], 
            physicsClientId=self.planningServer)


    def getObjectGlobalPose(self, local_pose, ee_global_pose):
        temp_object_global_pose = p.multiplyTransforms(
            ee_global_pose[0], ee_global_pose[1],
            local_pose[0], local_pose[1])
        object_global_pose = [list(temp_object_global_pose[0]), list(temp_object_global_pose[1])]

        return object_global_pose


    def checkPoseBasedOnConfig(self, pose, robot, workspace, armType, motionType, checkType):
        ### This function checks the validity of a pose by checking its corresponding config (IK)
        ### Input: pose: [[x,y,z],[x,y,z,w]]
        ###        armType: "Left" or "Right"
        ###        motionType: "transfer" or "transit" or "others"
        ###        checkType: "discrete" or "continuous"
        ### Output: isPoseValid (bool), singleArmConfig_IK (list (7-by-1))
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
                                maxNumIterations=2000, residualThreshold=0.0000001,
                                physicsClientId=robot.server)
        singleArmConfig_IK = config_IK[first_joint_index:first_joint_index+7]
        isPoseValid = self.checkIK(
                        singleArmConfig_IK, ee_idx, pose, robot, workspace, armType, motionType)


        trials = 0
        while (not isPoseValid) and (trials < 5):
            if checkType == "discrete":
                ### reset arm configuration
                self.IKresetForSingleArm(robot, armType)
            ### try another IK
            config_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                    endEffectorLinkIndex=ee_idx,
                                    targetPosition=pose[0],
                                    targetOrientation=pose[1],
                                    lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                                    maxNumIterations=2000, residualThreshold=0.0000001,
                                    physicsClientId=robot.server)
            singleArmConfig_IK = config_IK[first_joint_index:first_joint_index+7]
            isPoseValid = self.checkIK(
                singleArmConfig_IK, ee_idx, pose, robot, workspace, armType, motionType)
            if isPoseValid: break
            ### otherwise
            trials += 1
        ### you need to return both the statement whether the pose is valid
        ### and the valid configuration the pose corresponds to
        return isPoseValid, singleArmConfig_IK


    def checkIK(self, 
            singleArmConfig_IK, ee_idx, desired_ee_pose, robot, workspace, armType, motionType):
        ### This function checks if an IK solution is valid in terms of
        ### (1) small error from the desired pose (position error + quaternion error)
        ### (2) no collision with the robot and the workspace
        ### Input: desired_ee_pose: [[x,y,z], [x,y,z,w]]
        ###        armType: "Left" or "Right"
        ###        motionType: "transfer" or "transit" or "others"
        ### Output: isValid (bool) indicating whether the IK is valid
        robot.setSingleArmToConfig(singleArmConfig_IK, armType)

        ### If currently it is in hand manipulation, also move the object 
        if (self.isObjectInLeftHand and armType == "Left") or \
                            (self.isObjectInRightHand and armType == "Right"):
            self.updateRealObjectBasedonLocalPose(robot, armType)

        isValid = False

        if armType == "Left":
            actual_ee_pose = robot.left_ee_pose
        else:
            actual_ee_pose = robot.right_ee_pose
        ### first check if IK succeed
        ### if the ee_idx is within 2.0cm(0.02m) Euclidean distance from the desired one, we accept it
        ee_dist_pos = utils.computePoseDist_pos(actual_ee_pose[0], desired_ee_pose[0])
        if ee_dist_pos > 0.02:
            print("IK not reachable as position error exceeds 2cm")
            return isValid
        else:
            ### Now check orientation error
            ee_dist_quat = utils.computePoseDist_quat(actual_ee_pose[1], desired_ee_pose[1])
            if ee_dist_quat > 0.8:
                print("IK not reachable as quaternion error exceeds 0.8")
                return isValid

        ### Congrats! The IK success checker passed. Then check if there is collision
        isValid = self.checkIK_CollisionWithRobotAndKnownGEO(
                                            singleArmConfig_IK, robot, workspace, armType)
        if not isValid: return isValid
        ### depend on what type of motion it is, we have different collision check strategies
        ### IN TERMS OF THE OBJECT!
        if motionType == "transit":
            isValid = self.checkIK_CollisionWithStaticObject(
                                            singleArmConfig_IK, robot, workspace, armType)
        elif motionType == "transfer":
            isValid = self.checkIK_CollisionWithMovingObject(
                                            singleArmConfig_IK, robot, workspace, armType)

        # print("confirm ee_idx: ", ee_idx)
        # ee_pose = p.getLinkState(robot.motomanGEO, ee_idx, physicsClientId=robot.server)
        # tube_pose = p.getLinkState(robot.motomanGEO, 9, physicsClientId=robot.server)
        # print("ee_pose: ", ee_pose)
        # print("tube_pose: ", tube_pose)
        # print("let's look at this IK!!!!!!!!!!!")
        # time.sleep(10000)
        return isValid


    def checkIK_CollisionWithMovingObject(self, singleArmConfig_IK, robot, workspace, armType):
        robot.setSingleArmToConfig(singleArmConfig_IK, armType)
        isValid = False

        if armType == "Left":
            object_geometry = [self.objectInLeftHand]
        else:
            object_geometry = [self.objectInRightHand]

        ### check if there is collision between the robot and the moving object
        if self.collisionAgent_p.collisionCheck_robot_objectGEO(
            robot.motomanGEO, object_geometry, armType, 
            self.isObjectInLeftHand, self.isObjectInRightHand) == True:
            print("robot collide with moving object_geometry")
            return isValid
        else:
            pass
        ### In this case, you also need to check 
        ### if the moving object collides with known GEO (e.g., table)
        if self.collisionAgent_p.collisionCheck_object_knownGEO(
                                    object_geometry, workspace.known_geometries) == True:
            print("moving object collide with known geomtries")
            return isValid
        else:
            pass

        ### If you reach here, the configuration passes collision check with object geometry
        isValid = True
        # # print("pass IK collision checker with moving object geometry")
        return isValid


    def checkIK_CollisionWithStaticObject(self, singleArmConfig_IK, robot, workspace, armType):
        robot.setSingleArmToConfig(singleArmConfig_IK, armType)
        isValid = False
        ### check if there is collision between the robot and the object
        object_geometry = workspace.object_geometries.keys() ### list
        if self.collisionAgent_p.collisionCheck_robot_objectGEO(
            robot.motomanGEO, object_geometry, armType, 
            self.isObjectInLeftHand, self.isObjectInRightHand) == True:
            print("robot collide with static object_geometry")
            return isValid
        else:
            pass
        ### If you reach here, the configuration passes collision check with object geometry
        isValid = True
        # print("pass IK collision checker with static object geometry")
        return isValid        


    def checkIK_CollisionWithRobotAndKnownGEO(self, singleArmConfig_IK, robot, workspace, armType):
        robot.setSingleArmToConfig(singleArmConfig_IK, armType)
        isValid = False
        ### check if there is collision
        if self.collisionAgent_p.collisionCheck_selfCollision(robot.motomanGEO) == True:
            print("robot self collision")
            return isValid
        else:
            pass
        if self.collisionAgent_p.collisionCheck_robot_knownGEO(
                    robot.motomanGEO, workspace.known_geometries, armType) == True:
            print("robot collide with known geometries")
            return isValid
        else:
            pass
        ### If you reach here, the configuration passes collision check with known geometry
        isValid = True
        # print("pass IK collision checker with known GEO")
        return isValid


    def getTrajFromPath(self, path, initialPose, targetPose, robot, workspace, armType):
        ### This function converts a path (a list of indexes) to a trajectory (a list of joint-values)
        ### Input: path [start_index, idx2, ..., goal_index]
        ### Output: config_traj [[edge_config], [edge_config], ...]

        if armType == "Left":
            ee_idx = robot.left_ee_idx
            first_joint_index = 0
        else:
            ee_idx = robot.right_ee_idx
            first_joint_index = 7

        config_traj = [] ### a list of edge_configs
        for edge_idx in range(len(path)-1):
            config_edge_traj = []
            if edge_idx == 0:
                pose1 = initialPose
                pose2 = self.nodes[armType][path[edge_idx+1]]
                pose2 = [pose2[0:3], pose2[3:7]]
            elif edge_idx == len(path)-2:
                pose1 = self.nodes[armType][path[edge_idx]]
                pose1 = [pose1[0:3], pose1[3:7]]
                pose2 = targetPose
            else:
                pose1 = self.nodes[armType][path[edge_idx]]
                pose1 = [pose1[0:3], pose1[3:7]]
                pose2 = self.nodes[armType][path[edge_idx+1]]
                pose2 = [pose2[0:3], pose2[3:7]]

            min_dist = 0.01
            nseg = int(max(abs(pose1[0][0]-pose2[0][0]), 
                        abs(pose1[0][1]-pose2[0][1]), abs(pose1[0][2]-pose2[0][2])) / min_dist)
            if nseg == 0: nseg += 1
            # print("nseg: " + str(nseg))
            for i in range(1, nseg+1):
                interm_pos = utils.interpolatePosition(pose1[0], pose2[0], 1 / nseg * i)
                interm_quat = utils.interpolateQuaternion(pose1[1], pose2[1], 1 / nseg * i)
                interm_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                    endEffectorLinkIndex=ee_idx,
                                    targetPosition=interm_pos,
                                    targetOrientation=interm_quat,
                                    lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                                    maxNumIterations=2000, residualThreshold=0.0000001,
                                    physicsClientId=robot.server)
                interm_IK = interm_IK[first_joint_index:first_joint_index+7]
                config_edge_traj.append(interm_IK)
            
            ### finish the current edge
            config_traj.append(config_edge_traj)

        return config_traj


    def translate_between_poses(self, pose1, pose2, robot, workspace, armType):
        ### pose1, pose2: [[x,y,z], [x,y,z,w]]
        ### Output: config_traj = [[config_edge_traj]]
        config_edge_traj = [] ### a list of joint values
        if armType == "Left":
            ee_idx = robot.left_ee_idx
            first_joint_index = 0
        else:
            ee_idx = robot.right_ee_idx
            first_joint_index = 7
        min_dist = 0.01
        nseg = int(max(abs(pose1[0][0]-pose2[0][0]), 
                    abs(pose1[0][1]-pose2[0][1]), abs(pose1[0][2]-pose2[0][2])) / min_dist)
        if nseg == 0: nseg += 1
        print("nseg: " + str(nseg))

        for i in range(1, nseg):
            start_time = time.clock()
            interm_pos = utils.interpolatePosition(pose1[0], pose2[0], 1 / nseg * i)
            print("time for interpolatePosition: ", str(time.clock() - start_time))
            # start_time = time.clock()
            # interm_quat = utils.interpolateQuaternion(pose1[1], pose2[1], 1 / nseg * i)
            # print("time for interpolateQuaternion: ", str(time.clock() - start_time))
            # interm_pose = [interm_pos, interm_quat]
            start_time = time.clock()
            interm_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                    endEffectorLinkIndex=ee_idx,
                                    targetPosition=interm_pos,
                                    lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                                    maxNumIterations=2000, residualThreshold=0.0000001,
                                    physicsClientId=robot.server)
            print("time for IK calculation: ", str(time.clock() - start_time))
            interm_IK = interm_IK[first_joint_index:first_joint_index+7]
            config_edge_traj.append(interm_IK)

        return config_edge_traj


    def checkEdgeValidity_SGPoses(self, pose1, pose2, robot, workspace, armType, motionType):
        ### pose1, pose2: [[x,y,z], [x,y,z,w]]
        # if armType == "Left":
        #     ee_idx = robot.left_ee_idx
        #     first_joint_index = 0
        # else:
        #     ee_idx = robot.right_ee_idx
        #     first_joint_index = 7
        # nseg = 5
        config_edge_traj = [] ### a list of joint values
        min_dist = 0.01
        nseg = int(max(abs(pose1[0][0]-pose2[0][0]), 
                    abs(pose1[0][1]-pose2[0][1]), abs(pose1[0][2]-pose2[0][2])) / min_dist)
        if nseg == 0: nseg += 1
        # print("nseg: " + str(nseg))
        isEdgeValid = False
        ### no need to check two ends
        for i in range(1, nseg):
            interm_pos = utils.interpolatePosition(pose1[0], pose2[0], 1 / nseg * i)
            interm_quat = utils.interpolateQuaternion(pose1[1], pose2[1], 1 / nseg * i)
            interm_pose = [interm_pos, interm_quat]
            ### check this pose
            isPoseValid, pose_config = self.checkPoseBasedOnConfig(
                        interm_pose, robot, workspace, armType, motionType, "continuous")
            if not isPoseValid:
                config_edge_traj = []
                return isEdgeValid, config_edge_traj
            else:
                config_edge_traj.append(pose_config)

        ### Reach here because the edge is valid since all poses along the edge are valid
        isEdgeValid = True
        return isEdgeValid, config_edge_traj


    def checkEdgeValidity_cartesian(self, w1, w2, robot, workspace, armType):
        if armType == "Left":
            ee_idx = robot.left_ee_idx
        else:
            ee_idx = robot.right_ee_idx
        # nseg = 5
        min_dist = 0.01
        nseg = int(max(
            abs(w1[0][0]-w2[0][0]), abs(w1[0][1]-w2[0][1]), abs(w1[0][2]-w2[0][2])) / min_dist)
        if nseg == 0: nseg += 1
        # print("nseg: " + str(nseg))

        isEdgeValid = False
        for i in range(1, nseg):
            interm_pos = utils.interpolatePosition(w1[0], w2[0], 1 / nseg * i)
            interm_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                    endEffectorLinkIndex=ee_idx,
                                    targetPosition=interm_pos,
                                    lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                                    maxNumIterations=2000, residualThreshold=0.0000001,
                                    physicsClientId=robot.server)
            if armType == "Left":
                interm_IK = interm_IK[0:7]
            else:
                interm_IK = interm_IK[7:14]
            ### Then check if there is collision
            isValid = self.checkIK_CollisionWithRobotAndKnownGEO(interm_IK, robot, workspace, armType)
            if isValid == False:
                return isEdgeValid

        ### Reach here because the edge pass the collision check
        isEdgeValid = True
        return isEdgeValid


    def checkEdgeValidity(self, n1, n2, robot, workspace, armType):
        # nseg = 5
        min_degree = math.pi / 90
        nseg = int(max(
            abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]),
            abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6])) / min_degree)
        if nseg == 0: nseg += 1
        # print("nseg: " + str(nseg))

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
            ### Then check if there is collision
            isValid = self.checkIK_CollisionWithRobotAndKnownGEO(intermNode, robot, workspace, armType)
            if isValid == False:
                return isEdgeValid

        ### Reach here because the edge pass the collision check
        isEdgeValid = True
        return isEdgeValid


    def shortestPathPlanning(self, initialPose, targetPose, theme, robot, workspace, armType, motionType):
        ### Input: initialPose, targetPose [[x,y,z], [x,y,z,w]]
        ### first prepare the start_goal file
        f = self.writeStartGoal(initialPose[0], targetPose[0], theme)
        self.connectStartGoalToArmRoadmap(f,
                initialPose, targetPose, robot, workspace, armType, motionType)

        ### call the planning algorithm
        executeCommand = "./main_planner" + " " + theme + " " + armType + " " + str(self.nsamples) + " shortestPath"
        # subprocess.call(executeCommand, cwd="/home/rui/Documents/research/motoman_ws/src/pybullet_motoman/src", shell=True)
        rospack = RosPack()
        cwd = os.path.join(rospack.get_path("pybullet_motoman"), 'src')
        subprocess.call(executeCommand, cwd=cwd, shell=True)

        path = self.readPath(theme, armType)
        return path


    def readPath(self, theme, armType):
        path = []
        traj_file = os.path.join(self.rosPackagePath, "src/") + theme + "_traj.txt"
        f = open(traj_file, "r")
        nline = 0
        for line in f:
            nline += 1
            if nline == 1:
                line = line.split()
                isFailure = bool(int(line[0]))
                if isFailure:
                    f.close()
                    return path
            else:
                line = line.split()
                path = [int(e) for e in line]
                path.reverse()
        f.close()

        # for idx in path[1:-1]:
        #     traj.append(self.nodes[armType][idx])

        return path


    def writeStartGoal(self, initialPose, targetPose, theme):
        start_goal_file = os.path.join(self.rosPackagePath, "src/") + theme + ".txt"
        f = open(start_goal_file, "w")
        f.write(str(self.nsamples) + " ")
        for e in initialPose:
            f.write(str(e) + " ")
        f.write("\n")
        f.write(str(self.nsamples+1) + " ")
        for e in targetPose:
            f.write(str(e) + " ")
        f.write("\n")

        return f

    def connectStartGoalToArmRoadmap(
                        self, f, initialPose, targetPose, robot, workspace, armType, motionType):
        ### initialPose, targetPose [[x,y,z],[x,y,z,w]]
        ### if you trigger plan, it indicates the start and goal cannot be directly connected
        ### so in this case, we do not let start and goal be the neighbors
        start_id = self.nsamples
        target_id = self.nsamples + 1
        if armType == "Left":
            ee_idx = robot.left_ee_idx
        else:
            ee_idx = robot.right_ee_idx

        # neighborIndex_to_start = sorted(range(len(dist_to_start)), key=dist_to_start.__getitem__)

        dist_to_start = [
            utils.computePoseDist_pos(initialPose[0], curr_pose) for \
                                                curr_pose in self.workspaceNodes[armType]]
        neighborIndex_to_start, neighborDist_to_start = zip(
                                    *sorted(enumerate(dist_to_start), key=itemgetter(1)))
        neighborIndex_to_start = list(neighborIndex_to_start)
        neighborDist_to_start = list(neighborDist_to_start)

        dist_to_goal = [
            utils.computePoseDist_pos(targetPose[0], curr_pose) for \
                                                curr_pose in self.workspaceNodes[armType]]
        neighborIndex_to_goal, neighborDist_to_goal = zip(
                                    *sorted(enumerate(dist_to_goal), key=itemgetter(1)))
        neighborIndex_to_goal = list(neighborIndex_to_goal) 
        neighborDist_to_goal = list(neighborDist_to_goal)

        # num_neighbors = self.num_neighbors
        num_neighbors = 25

        ####### now connect potential neighbors for the start and the goal #######
        ### for start
        # print("for start")        
        neighbors_connected = 0
        for j in range(len(neighborIndex_to_start)):
            ### first check if the query node has already connected to enough neighbors
            if neighbors_connected >= num_neighbors:
                break
            ### otherwise, find the neighbor
            neighbor = self.nodes[armType][neighborIndex_to_start[j]]
            neighbor = [neighbor[0:3], neighbor[3:7]]
            ### check the edge validity
            isEdgeValid, config_edge_traj = self.checkEdgeValidity_SGPoses(
                        initialPose, neighbor, robot, workspace, armType, motionType)
            ### so far we do not record the config_edge_traj
            if isEdgeValid:
                f.write(str(start_id) + " " + str(neighborIndex_to_start[j]) + \
                                             " " + str(neighborDist_to_start[j]) + "\n")
                neighbors_connected += 1
        print("Number of neighbors for start node " + str(start_id) + ": " + str(neighbors_connected))

        ### for goal
        # print("for goal")
        neighbors_connected = 0
        for j in range(len(neighborIndex_to_goal)):        
            ### first check if the query node has already connected to enough neighbors
            if neighbors_connected >= num_neighbors:
                break
            ### otherwise, find the neighbor
            neighbor = self.nodes[armType][neighborIndex_to_goal[j]]
            neighbor = [neighbor[0:3], neighbor[3:7]]
            ### check the edge validity
            isEdgeValid, config_edge_traj = self.checkEdgeValidity_SGPoses(
                        neighbor, targetPose, robot, workspace, armType, motionType)
            ### so far we do not record the config_edge_traj
            if isEdgeValid:
                f.write(str(target_id) + " " + str(neighborIndex_to_goal[j]) + \
                                            " " + str(neighborDist_to_goal[j]) + "\n")            
                neighbors_connected += 1
        print("Number of neighbors for goal node " + str(target_id) + ": " + str(neighbors_connected))

        f.close()




############### the codes below are not used but kept for legacy ##############
# def jointMetric(self, a, b):
#     ### joint distance
#     dist = 0.0
#     for i in len(a):
#         dist += (a[i]-b[i])**2
#     return math.sqrt(dist)


# def cartesianMetric(self, a, b):
#     ### end effector distance in cartesian space
#     pos_dist = math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)
#     quat_inner_product = a[3]*b[3] + a[4]*b[4] + a[5]*b[5] + a[6]*b[6]
#     # print("pos_dist: ", pos_dist)
#     # print("quat_inner_product: ", quat_inner_product)
#     dist = pos_dist + self.weight_r*(1 - abs(quat_inner_product))
#     return dist


# def samplesConnect_notused(self, robot, workspace, armType):
#     connectivity = np.zeros((self.nsamples, self.nsamples))
#     knn = NearestNeighbors(
#         n_neighbors=self.num_neighbors, algorithm='auto', metric=lambda a,b: self.cartesianMetric(a,b))
#     knn.fit(self.workspaceNodes[armType])
#     neigh_dist, neigh_index = knn.kneighbors(
#                 X=self.workspaceNodes[armType], n_neighbors=self.num_neighbors, return_distance=True)
#     connectionsFile = self.roadmapFolder + "/connections_" + str(armType) + "_" + str(self.weight_option) + ".txt"
#     f_connection = open(connectionsFile, "w")
#     ### for each node
#     for i in range(len(self.nodes[armType])):
#         queryNode = self.nodes[armType][i]
#         neighbors_connected = 0
#         ### for each potential neighbor
#         for j in range(len(neigh_index[i])):
#             ### first check if this query node has already connected to enough neighbors
#             if neighbors_connected >= self.num_neighbors:
#                 break
#             if neigh_index[i][j] == i:
#                 ### if the neighbor is the query node itself
#                 continue
#             if connectivity[i][neigh_index[i][j]] == 1:
#                 ### the connectivity has been checked before
#                 neighbors_connected += 1
#                 continue
#             ### Otherwise, check the edge validity
#             ### in terms of collision with the robot itself and all known geometries (e.g. table/shelf)
#             ### between the query node and the current neighbor
#             neighbor = self.nodes[armType][neigh_index[i][j]]
#             isEdgeValid = self.checkEdgeValidity(queryNode, neighbor, robot, workspace, armType)
#             if isEdgeValid:
#                 ### write the edge information with their costs and labels into the txt file
#                 f_connection.write(str(i) + " " + str(neigh_index[i][j]) + " " + str(neigh_dist[i][j]) + "\n")
#                 connectivity[i][neigh_index[i][j]] = 1
#                 connectivity[neigh_index[i][j]][i] = 1
#                 neighbors_connected += 1
#         print("Number of neighbors for current node " + str(i) + ": " + str(neighbors_connected))
#     f_connection.close()


# def connectStartGoalToArmRoadmap_notused(self,
#     f, initialSingleArmConfig, targetSingleArmConfig, robot, workspace, armType):

#     startGoalConnect = False
#     start_id = self.nsamples
#     target_id = self.nsamples + 1
#     if armType == "Left":
#         ee_idx = robot.left_ee_idx
#     else:
#         ee_idx = robot.right_ee_idx

#     robot.setSingleArmToConfig(initialSingleArmConfig, armType)
#     temp_pose = p.getLinkState(robot.motomanGEO, ee_idx, physicsClientId=robot.server)
#     initialPose = list(temp_pose[0]) + list(temp_pose[1])
#     robot.setSingleArmToConfig(targetSingleArmConfig, armType)
#     temp_pose = p.getLinkState(robot.motomanGEO, ee_idx, physicsClientId=robot.server)
#     targetPose = list(temp_pose[0]) + list(temp_pose[1])
#     self.workspaceNodes[armType].append(initialPose)
#     self.workspaceNodes[armType].append(targetPose)

#     self.knn = NearestNeighbors(
#         n_neighbors=self.num_neighbors, algorithm='auto', metric=lambda a,b: self.cartesianMetric(a,b))
#     self.knn.fit(self.workspaceNodes[armType])
#     neigh_dist, neigh_index = knn.kneighbors(
#         X=self.workspaceNodes[armType], n_neighbors=self.num_neighbors, return_distance=True)

#     ############# connect the initialSingleArmConfig to the roadmap ###################
#     neighbors_connected = 0
#     ### for each potential neighbor
#     for j in range(len(neigh_index[-2])):
#         ### first check if the initialSingleArmConfig has already connected to enough neighbors
#         if neighbors_connected >= self.num_neighbors:
#             break
#         if neigh_index[-2][j] == start_id:
#             ### if the neighbor is the query node itself
#             continue
#         if neigh_index[-2][j] == target_id:
#             ### if the neighbor is the target
#             startGoalConnect = True
#         ### otherwise, check the edge validity
#         neighbor = self.nodes[armType][neigh_index[-2][j]]
#         isEdgeValid = self.checkEdgeValidity(initialSingleArmConfig, neighbor, robot, workspace, armType)
#         if isEdgeValid:
#             f.write(str(start_id) + " " + str(neigh_index[-2][j]) + " " + str(neigh_dist[-2][j]))
#             f.write("\n")
#             neighbors_connected += 1
#     print("Number of neighbors for start node " + str(start_id) + ": " + str(neighbors_connected))
#     ##############################################################################

#     ############# connect the targetSingleArmConfig to the roadmap ###################
#     neighbors_connected = 0
#     ### for each potential neighbor
#     for j in range(len(neigh_index[-1])):
#         ### first check if the targetSingleArmConfig has already connected to enough neighbors
#         if neighbors_connected >= self.num_neighbors:
#             break
#         if neigh_index[-1][j] == start_id:
#             ### if the neighbor is the start
#             if startGoalConnect == True: continue
#         if neigh_index[-1][j] == target_id:
#             ### if the neighbor is the query node itself
#             continue
#         ### otherwise, check the edge validity
#         neighbor = self.nodes[armType][neigh_index[-1][j]]
#         isEdgeValid = self.checkEdgeValidity(targetSingleArmConfig, neighbor, robot, workspace, armType)
#         if isValid:
#             f.write(str(target_id) + " " + str(neigh_index[-1][j]) + " " + str(neigh_dist[-1][j]))
#             f.write("\n")
#             neighbors_connected += 1
#     print("Number of neighbors for goal node " + str(target_id) + ": " + str(neighbors_connected))
#     ##############################################################################


#     # ### at the end, don't forget to delete initialPose and targetPose from the workspaceNodes
#     self.workspaceNodes[armType].remove(self.workspaceNodes[armType][-1])
#     self.workspaceNodes[armType].remove(self.workspaceNodes[armType][-1])
#     ##############################################################################

#     f.close()



# def connectStartGoalToArmRoadmap_old(
#                     self, f, initialPose, targetPose, robot, workspace, armType):
#     startGoalConnect = False
#     start_id = self.nsamples
#     target_id = self.nsamples + 1
#     if armType == "Left":
#         ee_idx = robot.left_ee_idx
#     else:
#         ee_idx = robot.right_ee_idx

#     self.workspaceNodes[armType].append(initialPose[0])
#     self.workspaceNodes[armType].append(targetPose[0])
#     tree = spatial.KDTree(self.workspaceNodes[armType])

#     ###### for start ######
#     start_workspaceNode = self.workspaceNodes[armType][-2]
#     knn = tree.query(start_workspaceNode, k=self.num_neighbors, p=2)
#     neighbors_connected = 0
#     ### for each potential neighbor
#     for j in range(len(knn[1])):
#         ### first check if this query node has already connected to enough neighbors
#         if neighbors_connected >= self.num_neighbors:
#             break
#         elif knn[1][j] == start_id:
#             ### if the neighbor is the start itself
#             continue
#         elif knn[1][j] == target_id:
#             ### if the neighbor is the target
#             startGoalConnect = True
#             neighbor = targetPose
#         else:
#             ### otherwise, find the neighbor
#             neighbor = self.nodes[armType][knn[1][j]]
#             neighbor = [neighbor[0:3], neighbor[3:7]]
#         ### check the edge validity
#         isEdgeValid = self.checkEdgeValidity_cartesian(initialPose, neighbor, robot, workspace, armType)
#         if isEdgeValid:
#             f.write(str(start_id) + " " + str(knn[1][j]) + " " + str(knn[0][j]) + "\n")
#             neighbors_connected += 1
#     print("Number of neighbors for start node " + str(start_id) + ": " + str(neighbors_connected))

#     ###### for goal ######
#     goal_workspaceNode = self.workspaceNodes[armType][-1]
#     knn = tree.query(goal_workspaceNode, k=self.num_neighbors, p=2)
#     neighbors_connected = 0
#     ### for each potential neighbor
#     for j in range(len(knn[1])):
#         ### first check if this query node has already connected to enough neighbors
#         if neighbors_connected >= self.num_neighbors:
#             break
#         elif knn[1][j] == target_id:
#             ### if the neighbor is the target itself
#             continue
#         elif knn[1][j] == start_id:
#             ### if the neighbor is the start
#             if startGoalConnect == True:
#                 continue
#             else:
#                 neighbor = initialPose
#         else:
#             ### otherwise, find the neighbor
#             neighbor = self.nodes[armType][knn[1][j]]
#             neighbor = [neighbor[0:3], neighbor[3:7]]
#         ### check the edge validity
#         isEdgeValid = self.checkEdgeValidity_cartesian(targetPose, neighbor, robot, workspace, armType)
#         if isEdgeValid:
#             f.write(str(target_id) + " " + str(knn[1][j]) + " " + str(knn[0][j]) + "\n")
#             neighbors_connected += 1
#     print("Number of neighbors for goal node " + str(target_id) + ": " + str(neighbors_connected))

#     ### at the end, don't forget to delete initialPose and targetPose from the workspaceNodes
#     self.workspaceNodes[armType].remove(self.workspaceNodes[armType][-1])
#     self.workspaceNodes[armType].remove(self.workspaceNodes[armType][-1])
#     ##############################################################################

#     f.close()
