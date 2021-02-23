from __future__ import division
from __future__ import print_function
import pybullet as p
import pybullet_data

import os
import random
import shutil
import math
import numpy as np
import scipy
import scipy.optimize as sop
from scipy import spatial
import time
import IPython
import subprocess
from operator import itemgetter
import copy
import json
# import sklearn
# from sklearn.neighbors import NearestNeighbors

import utils
from CollisionChecker import CollisionChecker

import rospy
from rospkg import RosPack


from trac_ik_python.trac_ik import IK
from tf import TransformListener
from geometry_msgs.msg import Pose, PoseStamped

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

        self.isObjectInLeftHand = isObjectInLeftHand
        self.isObjectInRightHand = isObjectInRightHand
        self.objectInLeftHand = objectInLeftHand
        self.objectInRightHand = objectInRightHand
        self.leftLocalPose = [[-1, -1, -1], [-1, -1, -1, -1]]
        self.rightLocalPose = [[-1, -1, -1], [-1, -1, -1, -1]]

        self.tf = None
        self.left_ik_solver = IK('torso_link_b1', 'motoman_left_ee', timeout=.05, epsilon=1e-5)
        self.right_ik_solver  = IK('torso_link_b1', 'motoman_right_ee', timeout=.05, epsilon=1e-5)
        self.rp_db = self.readRestposeFromJson("joint_to_pose.json")

    def readRestposeFromJson(self, fileName):
        file_path = os.path.join(self.rosPackagePath, "script", "joint_to_pose.json")
        f = open(file_path, 'r')
        res = json.load(f)
        f.close()
        return res

    def inverse_kinematics(self, arm_type, pose, rest_pose):
        """
        input: arm_type, pose, rest_pose (length of 7)
        output: joint angles (length of 7), or None if failed
        """
        # rospy.sleep(1000000000000.)
        # better create a solver in the object
        if self.tf is None:
            # rospy.sleep(10000000000000000.)
            self.tf = TransformListener()
        if arm_type == 'Left':
            ik_solver = self.left_ik_solver
            start_joint_idx = 0
        else:
            ik_solver = self.right_ik_solver
            start_joint_idx = 7
        
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = pose[0][0]
        pose_msg.pose.position.y = pose[0][1]
        pose_msg.pose.position.z = pose[0][2]
        pose_msg.pose.orientation.x = pose[1][0]
        pose_msg.pose.orientation.y = pose[1][1]
        pose_msg.pose.orientation.z = pose[1][2]
        pose_msg.pose.orientation.w = pose[1][3]
        
        self.tf.waitForTransform('torso_link_b1', 'base_link', rospy.Time(), rospy.Duration(5))
        transformed_pose = self.tf.transformPose('torso_link_b1', pose_msg)

        if len(rest_pose) == 7:
            seed = np.array(rest_pose)
        else:
            seed = np.array(rest_pose[start_joint_idx:start_joint_idx+7])
        sol = ik_solver.get_ik(seed, transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z, \
                               transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y, transformed_pose.pose.orientation.z, transformed_pose.pose.orientation.w)
        if sol is None:
            return None
        else:
            return list(sol)

    def linear_inverse_kinematics(self, arm_type, pose, rest_pose, robot, start_pose=None):
        """
        linearly interpolate between start and goal poses, and do IK at each intermediate nodes.
        use the final solution as the joint state for the goal
        """
        if arm_type == 'Left':
            ee_idx = robot.left_ee_idx
            start_joint_idx = 0
        else:
            ee_idx = robot.right_ee_idx
            start_joint_idx = 7
        if len(rest_pose) != 7:
            rest_pose = rest_pose[start_joint_idx:start_joint_idx+7]
        if start_pose is None:
            # get start pose for rest_pose
            robot.setSingleArmToConfig(rest_pose, arm_type)
            res = p.getLinkState(robot.motomanGEO, ee_idx, physicsClientId=robot.server)
            start_pose = [res[0], res[1]]
        
        # linearly interpolate to start_pose
        min_dist = 0.02
        pose1 = start_pose
        pose2 = pose
        nseg = int(max(abs(pose1[0][0]-pose2[0][0]), 
                    abs(pose1[0][1]-pose2[0][1]), abs(pose1[0][2]-pose2[0][2])) / min_dist)
        if nseg == 0: nseg += 1
        new_rest_pose = None
        for i in range(nseg):
            interm_pos = utils.interpolatePosition(pose1[0], pose2[0], 1 / nseg * i)
            interm_quat = utils.interpolateQuaternion(pose1[1], pose2[1], 1 / nseg * i)
            interm_pose = [interm_pos, interm_quat]
            # compute IK
            new_rest_pose = self.inverse_kinematics(arm_type, interm_pose, rest_pose)
            if new_rest_pose is None:
                continue
            else:
                rest_pose = new_rest_pose
        # perform last inverse kinemaitcs using the latest rest pose
        new_rest_pose = self.inverse_kinematics(arm_type, pose, rest_pose)
        return new_rest_pose

    def loadSamples(self):
        arms = ["Left", "Right"]
        for armType in arms:
            samplesFile = self.roadmapFolder + "/samples_" + str(armType) + ".txt"

            f_samples = open(samplesFile, "r")
            for line in f_samples:
                line = line.split()
                line = [float(e) for e in line[1:]]
                self.nodes[armType].append(line)
            f_samples.close()

        self.nsamples = len(self.nodes["Left"])

        ### specify the needed parameters
        self.neighbors_const = 2.5 * math.e * (1 + 1/len(self.nodes["Left"][0]))
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
            if armType == "Left":
                ee_idx = robot.left_ee_idx
            else:
                ee_idx = robot.right_ee_idx
            self.samplingNodes(ee_idx, robot, workspace, armType)
            self.saveSamplesToFile(samplesFile, armType)

        ### specify the needed parameters
        self.neighbors_const = 2.5 * math.e * (1 + 1/len(self.nodes["Left"][0]))
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
            ikSolution = self.singleSampling_CSpace(robot, armType) ### this is for a single arm
            ### check if the IK solution is valid in terms of
            ### no collision with the robot and other known geometries like table/shelf/etc..
            isValid = self.checkIK_CollisionWithRobotAndKnownGEO(ikSolution, robot, workspace, armType)
            if isValid:
                self.nodes[armType].append(ikSolution)
                temp_counter += 1


    def singleSampling_CSpace(self, robot, armType):
        if armType == "Left":
            first_joint_index = 0
        else:
            first_joint_index = 7
        numJoints = int(len(robot.leftArmHomeConfiguration))
        ikSolution = []
        for i in range(numJoints):
            ikSolution.append(
                random.uniform(robot.ll[first_joint_index+i], robot.ul[first_joint_index+i]))

        return ikSolution


    def saveSamplesToFile(self, samplesFile, armType):
        f_samples = open(samplesFile, "w")
        for node_idx in range(len(self.nodes[armType])):
            node = self.nodes[armType][node_idx]
            f_samples.write(str(node_idx))
            for k in range(len(node)):
                f_samples.write(" " + str(node[k]))
            f_samples.write("\n")
        f_samples.close()


    def samplesConnect(self, robot, workspace, armType):
        connectivity = np.zeros((self.nsamples, self.nsamples))
        tree = spatial.KDTree(self.nodes[armType]) ### use KD tree to arrange neighbors assignment
        connectionsFile = self.roadmapFolder + "/connections_" + str(armType) + ".txt"
        f_connection = open(connectionsFile, "w")
        ### for each node
        for node_idx in range(len(self.nodes[armType])):
            queryNode = self.nodes[armType][node_idx]
            knn = tree.query(queryNode, k=self.num_neighbors, p=2)

            neighbors_connected = 0
            ### for each potential neighbor
            for j in range(len(knn[1])):
                ### first check if this query node has already connected to enough neighbors
                if neighbors_connected >= self.num_neighbors:
                    break
                if knn[1][j] == node_idx:
                    ### if the neighbor is the query node itself
                    continue
                if connectivity[node_idx][knn[1][j]] == 1:
                    ### the connectivity has been checked before
                    neighbors_connected += 1
                    continue
                ### Otherwise, check the edge validity
                ### in terms of collision with the robot itself and all known geometries (e.g. table/shelf)
                ### between the query node and the current neighbor
                neighbor = self.nodes[armType][knn[1][j]]
                isEdgeValid = self.checkEdgeValidity_knownGEO(queryNode, neighbor, robot, workspace, armType)
                if isEdgeValid:
                    ### write this edge information with their costs and labels into the txt file
                    f_connection.write(str(node_idx) + " " + str(knn[1][j]) + " " + str(knn[0][j]) + "\n")
                    connectivity[node_idx][knn[1][j]] = 1
                    connectivity[knn[1][j]][node_idx] = 1
                    neighbors_connected += 1
            print("Number of neighbors for current node " + str(node_idx) + ": " + str(neighbors_connected))
        f_connection.close()


    def updateManipulationStatus(self, 
        isObjectInLeftHand, isObjectInRightHand, object_geometries, robot):
        ### This update update the manipulation status by indicating
        ### whether the object is in any of the hand
        self.isObjectInLeftHand = isObjectInLeftHand
        self.isObjectInRightHand = isObjectInRightHand

        if self.isObjectInLeftHand:
            self.objectInLeftHand = object_geometries.keys()[0]
            self.leftLocalPose = self.computeLocalPose(
                        object_geometries.values()[0][0], robot, "Left")
        else:
            self.objectInLeftHand = None
            self.leftLocalPose = [[-1, -1, -1], [-1, -1, -1, -1]]

        if self.isObjectInRightHand:
            self.objectInRightHand = object_geometries.keys()[0]
            self.rightLocalPose = self.computeLocalPose(
                        object_geometries.values()[0][0], robot, "Right")
        else:
            self.objectInRightHand = None
            self.rightLocalPose = [[-1, -1, -1], [-1, -1, -1, -1]]


    def computeLocalPose(self, object_pose, robot, armType):
        ### This function computes local pose given current object_pose + ee pose
        ### when it is in-hand manipulation (the object is attached to the hand)
        if armType == "Left":
            curr_ee_pose = robot.left_ee_pose
        else:
            curr_ee_pose = robot.right_ee_pose

        inverse_ee_global = p.invertTransform(curr_ee_pose[0], curr_ee_pose[1])
        temp_localPose = p.multiplyTransforms(
            list(inverse_ee_global[0]), list(inverse_ee_global[1]),
            object_pose[0], object_pose[1])

        temp_localPose = [list(temp_localPose[0]), list(temp_localPose[1])]

        return temp_localPose


    def generatePreGrasp(self, grasp_pose, robot, workspace, armType, motionType):
        ### This function generates a pre-grasp pose based on grasp pose
        ### It is 10cm (0.1m) behind the approaching direction of z axis (local axis of the end effector)
        ### Input: grasp_pose: [[x,y,z], [x,y,z,w]]
        if armType == "Left":
            ee_idx = robot.left_ee_idx
            first_joint_index = 0
            db_start_idx = 0
        else:
            ee_idx = robot.right_ee_idx
            first_joint_index = 7
            db_start_idx = 12

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
                                lowerLimits=robot.ll, upperLimits=robot.ul, 
                                jointRanges=robot.jr, restPoses=robot.rp,
                                maxNumIterations=10000, residualThreshold=0.001,
                                physicsClientId=robot.server)
        singleArmConfig_IK = list(q_preGraspIK[first_joint_index:first_joint_index+7])
        singleArmConfig_IK, isIKValid = self.AdjustIKBasedOnJointLimit(singleArmConfig_IK, robot, armType)
        isPoseValid = False
        if isIKValid:
            isPoseValid = self.checkPoseIK(singleArmConfig_IK, ee_idx, preGrasp_pose, robot, workspace, armType, motionType)
        if not isPoseValid:
            print('Pybullet failed. Linear IK')
            singleArmConfig_IK = self.linear_inverse_kinematics(armType, preGrasp_pose, robot.rp, robot, start_pose=None)
            isIKValid = False
            isPoseValid = False
            if singleArmConfig_IK is not None:
                singleArmConfig_IK, isIKValid = self.AdjustIKBasedOnJointLimit(singleArmConfig_IK, robot, armType)
                if isIKValid:
                    isPoseValid = self.checkPoseIK(
                            singleArmConfig_IK, ee_idx, preGrasp_pose, robot, workspace, armType, motionType)

        # if not isPoseValid:
        #     # singleArmConfig_IK = self.inverse_kinematics(armType, preGrasp_pose, robot.rp)
        #     # print("show me current rest pose of the robot: ")
        #     # print(robot.rp)
        #     singleArmConfig_IK = self.linear_inverse_kinematics(armType, preGrasp_pose, robot.rp, robot, start_pose=None)
        #     isPoseValid = False
        #     if singleArmConfig_IK is not None:
        #         isPoseValid = self.checkPoseIK(
        #                 singleArmConfig_IK, ee_idx, preGrasp_pose, robot, workspace, armType, motionType)

        trials = db_start_idx
        while (not isPoseValid) and (trials < len(self.rp_db)):
            ### try another IK (not specify rest pose)
            # rp = self.randomizeRestposes(robot, armType)
            print('IK trial %d...' % (trials))
            rp = self.useRestPoseFromJsonFile(robot, armType, trials)
            q_preGraspIK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                    endEffectorLinkIndex=ee_idx,
                                    targetPosition=preGrasp_pose[0],
                                    targetOrientation=preGrasp_pose[1],
                                    lowerLimits=robot.ll, upperLimits=robot.ul, 
                                    jointRanges=robot.jr, restPoses=rp,
                                    maxNumIterations=10000, residualThreshold=0.001,
                                    physicsClientId=robot.server)
            singleArmConfig_IK = list(q_preGraspIK[first_joint_index:first_joint_index+7])
            singleArmConfig_IK, isIKValid = self.AdjustIKBasedOnJointLimit(singleArmConfig_IK, robot, armType)
            isPoseValid = False
            if isIKValid:
                isPoseValid = self.checkPoseIK(singleArmConfig_IK, ee_idx, preGrasp_pose, robot, workspace, armType, motionType)
            if not isPoseValid:
                print('Pybullet failed. Linear IK')
                singleArmConfig_IK = self.linear_inverse_kinematics(armType, preGrasp_pose, rp, robot, start_pose=None)
                isIKValid = False
                isPoseValid = False
                if singleArmConfig_IK is not None:
                    singleArmConfig_IK, isIKValid = self.AdjustIKBasedOnJointLimit(singleArmConfig_IK, robot, armType)
                    if isIKValid:
                        isPoseValid = self.checkPoseIK(singleArmConfig_IK, ee_idx, preGrasp_pose, robot, workspace, armType, motionType)
            if isIKValid and isPoseValid:
                break
            else:
                trials += 1
        ### you need to return both the statement whether the pose is valid
        ### and the valid configuration the pose corresponds to
        return isPoseValid, preGrasp_pose, singleArmConfig_IK

    def randomizeRestposes(self, robot, armType):
        rp = copy.deepcopy(robot.rp)
        if armType == "Left":
            start_joint_index = 0
        else:
            start_joint_index = 7
        for i in range(7):
            rp[i+start_joint_index] = rp[i+start_joint_index] + random.uniform(-10*math.pi/180, 10*math.pi/180)
            if rp[i+start_joint_index] < robot.ll[i+start_joint_index]:
                rp[i+start_joint_index] = robot.ll[i+start_joint_index]
            if rp[i+start_joint_index] > robot.ul[i+start_joint_index]:
                rp[i+start_joint_index] = robot.ul[i+start_joint_index]
    
        return rp;

    
    def useRestPoseFromJsonFile(self, robot, armType, idx):
        rest_pose = []
        for i in range(14):
            rest_pose.append(self.rp_db[idx]['joint'][robot.motomanRJointNames[i]])
        return rest_pose

    def AdjustIKBasedOnJointLimit(self, singleArmConfig, robot, armType):

        singleArmConfig = list(singleArmConfig)

        isIKValid = False

        if armType == "Left":
            start_joint_index = 0
        else:
            start_joint_index = 7
        for joint_idx in range(len(singleArmConfig)):
            joint_value = singleArmConfig[joint_idx]
            # print("joint value: ", joint_value)
            while (joint_value < -math.pi):
                # print("incrementing")
                joint_value += 2*math.pi
            while (joint_value > math.pi):
                # print("decrementing")
                joint_value -= 2*math.pi
            if (joint_value < robot.ll[joint_idx+start_joint_index]) or (joint_value > robot.ul[joint_idx+start_joint_index]):
                return singleArmConfig, isIKValid
            else:
                singleArmConfig[joint_idx] = joint_value
            
        ## you reach here since it pass all joints check
        isIKValid = True
        return singleArmConfig, isIKValid
                

    def updateRealObjectBasedonLocalPose(self, robot, workspace, armType):
        if armType == "Left":
            # ee_idx = robot.left_ee_idx
            # objectInHand = self.objectInLeftHand
            # curr_ee_pose = robot.left_ee_pose
            # object_global_pose = self.getObjectGlobalPose(self.leftLocalPose, curr_ee_pose)
            object_global_pose = self.getObjectGlobalPose(self.leftLocalPose, robot.left_ee_pose)
            p.resetBasePositionAndOrientation(
                self.objectInLeftHand, object_global_pose[0], object_global_pose[1], 
                physicsClientId=self.planningServer)
            p.stepSimulation(physicsClientId=self.planningServer)
            workspace.object_geometries[self.objectInLeftHand][0] = [object_global_pose[0], object_global_pose[1]]

        else:
            # ee_idx = robot.right_ee_idx
            # objectInHand = self.objectInRightHand
            # curr_ee_pose = robot.right_ee_pose
            # object_global_pose = self.getObjectGlobalPose(self.rightLocalPose, curr_ee_pose)
            object_global_pose = self.getObjectGlobalPose(self.rightLocalPose, robot.right_ee_pose)
            p.resetBasePositionAndOrientation(
                self.objectInRightHand, object_global_pose[0], object_global_pose[1], 
                physicsClientId=self.planningServer)
            p.stepSimulation(physicsClientId=self.planningServer)
            workspace.object_geometries[self.objectInRightHand][0] = [object_global_pose[0], object_global_pose[1]]

        # p.resetBasePositionAndOrientation(
        #     objectInHand, object_global_pose[0], object_global_pose[1], 
        #     physicsClientId=self.planningServer)


    def getObjectGlobalPose(self, local_pose, ee_global_pose):
        temp_object_global_pose = p.multiplyTransforms(
            ee_global_pose[0], ee_global_pose[1],
            local_pose[0], local_pose[1])
        object_global_pose = [list(temp_object_global_pose[0]), list(temp_object_global_pose[1])]

        return object_global_pose

    ### TODO: (1) run baseline experiments
    ###       (2) make simulation run
    def generateConfigBasedOnPose(self, pose, robot, workspace, armType, motionType):
        ### This function checks the validity of a pose by checking its corresponding config (IK)
        ### Input: pose: [[x,y,z],[x,y,z,w]]
        ###        armType: "Left" or "Right"
        ###        motionType: "transfer" or "transit" or "reset" or 
        ###                    "moveAway" or "approachToPlacement"
        ### Output: isPoseValid (bool), singleArmConfig_IK (list (7-by-1))
        if armType == "Left":
            ee_idx = robot.left_ee_idx
            first_joint_index = 0
            db_start_idx = 0
        else:
            ee_idx = robot.right_ee_idx
            first_joint_index = 7
            db_start_idx = 12

        # if armType == "Right" and motionType == "transit":
        #     robot.rp[7:14] = [0.43908920097926757, 1.1427753607637277, 0.15512427391062386, -0.5407061993381723, 0.1436811090335915, -1.4603468659725638, 1.061848742243989]

        ### we add rest pose in the IK solver to get as high-quality IK as possible
        config_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                endEffectorLinkIndex=ee_idx,
                                targetPosition=pose[0],
                                targetOrientation=pose[1],
                                lowerLimits=robot.ll, upperLimits=robot.ul, 
                                jointRanges=robot.jr, restPoses=robot.rp,
                                maxNumIterations=20000, residualThreshold=0.001,
                                physicsClientId=robot.server)

        singleArmConfig_IK = list(config_IK[first_joint_index:first_joint_index+7])
        print("singleArmConfig_IK from pybullet: ", singleArmConfig_IK)
        singleArmConfig_IK, isIKValid = self.AdjustIKBasedOnJointLimit(singleArmConfig_IK, robot, armType)

        isPoseValid = False
        if isIKValid:
            print("yeah! At least IK is valid! Nice job pybullet")
            isPoseValid = self.checkPoseIK(
                    singleArmConfig_IK, ee_idx, pose, robot, workspace, armType, motionType)
        
        if not isPoseValid:
            # singleArmConfig_IK = self.inverse_kinematics(armType, pose, robot.rp)
            print('Pybullet failed. Linear IK')
            singleArmConfig_IK = self.linear_inverse_kinematics(armType, pose, robot.rp, robot, start_pose=None)
            isIKValid = False
            isPoseValid = False
            if singleArmConfig_IK is not None:
                print('linear ik solution is not None')
                singleArmConfig_IK, isIKValid = self.AdjustIKBasedOnJointLimit(singleArmConfig_IK, robot, armType)
                if isIKValid:
                    print('linear ik solution valid! Nice job our IK')
                    isPoseValid = self.checkPoseIK(
                            singleArmConfig_IK, ee_idx, pose, robot, workspace, armType, motionType)
        
        trials = db_start_idx
        while (not isPoseValid) and (trials < len(self.rp_db)):
            print('ik trial %d...' % (trials))
            ### try another IK (not specify rest pose)
            # rp = self.randomizeRestposes(robot, armType)
            rp = self.useRestPoseFromJsonFile(robot, armType, trials)
            config_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                    endEffectorLinkIndex=ee_idx,
                                    targetPosition=pose[0],
                                    targetOrientation=pose[1],
                                    lowerLimits=robot.ll, upperLimits=robot.ul, 
                                    jointRanges=robot.jr, restPoses = rp,
                                    maxNumIterations=20000, residualThreshold=0.001,
                                    physicsClientId=robot.server)
            singleArmConfig_IK = list(config_IK[first_joint_index:first_joint_index+7])
            singleArmConfig_IK, isIKValid = self.AdjustIKBasedOnJointLimit(singleArmConfig_IK, robot, armType)
            isPoseValid = False
            if isIKValid:
                isPoseValid = self.checkPoseIK(
                        singleArmConfig_IK, ee_idx, pose, robot, workspace, armType, motionType)
            if not isPoseValid:
                print('Pybullet failed. Linear IK')
                singleArmConfig_IK = self.linear_inverse_kinematics(armType, pose, rp, robot, start_pose=None)
                isIKValid = False
                isPoseValid = False 
                if singleArmConfig_IK is not None:
                    print('linear ik solution is not None')
                    singleArmConfig_IK, isIKValid = self.AdjustIKBasedOnJointLimit(singleArmConfig_IK, robot, armType)
                    if isIKValid:
                        print('linear ik solution valid! Nice job our IK')
                        isPoseValid = self.checkPoseIK(
                                singleArmConfig_IK, ee_idx, pose, robot, workspace, armType, motionType)
            if isIKValid and isPoseValid:
                break
            else:
                trials += 1
                continue
        ### you need to return both the statement whether the pose is valid
        ### and the valid configuration the pose corresponds to
        return isPoseValid, singleArmConfig_IK


    def checkIK_reachability(self, singleArmConfig_IK, desired_ee_pose, robot, armType):
        ### This function purely checks if an IK solution is reachable
        ### small position and orientation error
        robot.setSingleArmToConfig(singleArmConfig_IK, armType)

        isValid = False

        if armType == "Left":
            actual_ee_pose = copy.deepcopy(robot.left_ee_pose)
        else:
            actual_ee_pose = copy.deepcopy(robot.right_ee_pose)

        ### if the ee_idx is within 2.0cm(0.02m) Euclidean distance from the desired one, we accept it
        ee_dist_pos = utils.computePoseDist_pos(actual_ee_pose[0], desired_ee_pose[0])
        # print("actual_ee_pose: ", actual_ee_pose[0])
        # print("desired_ee_pose: ", desired_ee_pose[0])
        if ee_dist_pos > 0.02:
        # if ee_dist_pos > 0.03:
            print("position error: ", ee_dist_pos)
            print("IK not reachable as position error exceeds 2cm: " + str(ee_dist_pos))
            return isValid
        else:
            ### Now check orientation error
            ee_dist_quat = utils.computePoseDist_quat(actual_ee_pose[1], desired_ee_pose[1])
            if ee_dist_quat > 0.8:
                print("IK not reachable as quaternion error exceeds 0.8: " + str(ee_dist_quat))
                return isValid
        ### otherwise, this IK is reachable
        isValid = True
        return isValid


    def checkIK_AllCollisions(self, singleArmConfig_IK, robot, workspace, armType, motionType):
        ### This function checks all collisions based on different motionType
        ### Common: no robot self collision and collsions between robot and knownGEO AT ALL TIME
        ### no other collisions based on motionType
        ###     (i) "transit": no collision between the robot and the static object
        ###     (ii) "transfer" or "approachToPlacement": 
        ###           no collision between the robot and the moving object
        ###           no collision between the moving object and knownGEO
        isValid = self.checkIK_CollisionWithRobotAndKnownGEO(
                                            singleArmConfig_IK, robot, workspace, armType)
        if not isValid: return isValid

        ### If currently it is in hand manipulation, also move the object 
        if (self.isObjectInLeftHand and armType == "Left") or \
                            (self.isObjectInRightHand and armType == "Right"):
            self.updateRealObjectBasedonLocalPose(robot, workspace, armType)
    

        ### depend on what type of motion it is, we have different collision check strategies
        ### IN TERMS OF THE OBJECT!
        if motionType == "transit":
            isValid = self.checkIK_CollisionWithStaticObject(
                                            singleArmConfig_IK, robot, workspace, armType)
        elif motionType == "transfer":
            isValid = self.checkIK_CollisionWithMovingObject(
                                            singleArmConfig_IK, robot, workspace, armType)

        return isValid


    def checkPoseIK(self, 
            singleArmConfig_IK, ee_idx, desired_ee_pose, robot, workspace, armType, motionType):
        ### This function checks if an IK solution is valid in terms of
        ### (1) small error from the desired pose (position error + quaternion error)
        ### (2) no collision occurred (all types of collision based on motionType)

        ### Input: desired_ee_pose: [[x,y,z], [x,y,z,w]]
        ###        armType: "Left" or "Right"
        ###        motionType: "transfer" or "transit" or "reset"
        ### Output: isValid (bool) indicating whether the IK is valid

        isValid = self.checkIK_reachability(singleArmConfig_IK, desired_ee_pose, robot, armType)
        if not isValid: return isValid

        ### Congrats! The IK successfully passed reachability checker. 

        ### Then check if there is collision
        isValid = self.checkIK_AllCollisions(
                                singleArmConfig_IK, robot, workspace, armType, motionType)

        return isValid


    def checkIK_CollisionWithMovingObject(self, singleArmConfig_IK, robot, workspace, armType):
        # robot.setSingleArmToConfig(singleArmConfig_IK, armType)
        isValid = False

        if armType == "Left":
            object_geometry = [self.objectInLeftHand]
            # print("object_geometry")
            # print(object_geometry)
            # print("tell me object info at left hand: ")
            # print(workspace.object_geometries[self.objectInLeftHand])
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
            # print(singleArmConfig_IK)
            print("robot self collision")
            return isValid

        if self.collisionAgent_p.collisionCheck_robot_knownGEO(
                    robot.motomanGEO, workspace.known_geometries, armType) == True:
            print("robot collide with known geometries")
            return isValid


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
                                    lowerLimits=robot.ll, upperLimits=robot.ul, 
                                    jointRanges=robot.jr, restPoses=robot.rp,
                                    maxNumIterations=2000, residualThreshold=0.001,
                                    physicsClientId=robot.server)
                interm_IK = list(interm_IK[first_joint_index:first_joint_index+7])


                config_edge_traj.append(interm_IK)
            
            ### finish the current edge
            config_traj.append(config_edge_traj)

        return config_traj


    # def translate_move_configs(self, config1, config2, robot, workspace, armType):
    #     ### config1, config2: [q1, q2, ...]
    #     ### Output: config_edge_traj
    #     config_edge_traj = [] ### a list of joint values
    #     if armType == "Left":
    #         ee_idx = robot.left_ee_idx
    #         first_joint_index = 0
    #     else:
    #         ee_idx = robot.right_ee_idx
    #         first_joint_index = 7
    #     min_dist = 0.01
    #     nseg = int(max(abs(pose1[0][0]-pose2[0][0]), 
    #                 abs(pose1[0][1]-pose2[0][1]), abs(pose1[0][2]-pose2[0][2])) / min_dist)
    #     if nseg == 0: nseg += 1
    #     print("nseg: " + str(nseg))

    #     for i in range(1, nseg+1):
    #         start_time = time.clock()
    #         interm_pos = utils.interpolatePosition(pose1[0], pose2[0], 1 / nseg * i)
    #         print("time for interpolate position: ", str(time.clock() - start_time))
    #         # interm_quat = utils.interpolateQuaternion(pose1[1], pose2[1], 1 / nseg * i)
    #         start_time = time.clock()
    #         interm_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
    #                                 endEffectorLinkIndex=ee_idx,
    #                                 targetPosition=interm_pos,
    #                                 lowerLimits=robot.ll, upperLimits=robot.ul, 
    #                                 jointRanges=robot.jr, restPoses=robot.rp,
    #                                 maxNumIterations=20000, residualThreshold=0.0000001,
    #                                 physicsClientId=robot.server)
    #         print("time for IK: ", str(time.clock() - start_time))
    #         # print("time for IK calculation: ", str(time.clock() - start_time))
    #         interm_IK = list(interm_IK[first_joint_index:first_joint_index+7])
    #         config_edge_traj.append(interm_IK)

    #     return config_edge_traj


    # def checkEdgeValidity_SGPoses(self, pose1, pose2, robot, workspace, armType, motionType):
    #     ### pose1, pose2: [[x,y,z], [x,y,z,w]]
    #     # if armType == "Left":
    #     #     ee_idx = robot.left_ee_idx
    #     #     first_joint_index = 0
    #     # else:
    #     #     ee_idx = robot.right_ee_idx
    #     #     first_joint_index = 7
    #     # nseg = 5
    #     config_edge_traj = [] ### a list of joint values
    #     min_dist = 0.01
    #     nseg = int(max(abs(pose1[0][0]-pose2[0][0]), 
    #                 abs(pose1[0][1]-pose2[0][1]), abs(pose1[0][2]-pose2[0][2])) / min_dist)
    #     if nseg == 0: nseg += 1
    #     # print("nseg: " + str(nseg))
    #     isEdgeValid = False
    #     ### no need to check two ends
    #     for i in range(1, nseg):
    #         interm_pos = utils.interpolatePosition(pose1[0], pose2[0], 1 / nseg * i)
    #         interm_quat = utils.interpolateQuaternion(pose1[1], pose2[1], 1 / nseg * i)
    #         interm_pose = [interm_pos, interm_quat]
    #         ### check this pose
    #         isPoseValid, pose_config = self.checkPoseBasedOnConfig(
    #                     interm_pose, robot, workspace, armType, motionType, "continuous")
    #         if not isPoseValid:
    #             config_edge_traj = []
    #             return isEdgeValid, config_edge_traj
    #         else:
    #             config_edge_traj.append(pose_config)

    #     ### Reach here because the edge is valid since all poses along the edge are valid
    #     isEdgeValid = True
    #     return isEdgeValid, config_edge_traj


    def checkEdgeValidity_cartesianMove(self, initialConfig, pose1, pose2, robot, workspace, armType, motionType):
        ### set the rest pose to initialConfig
        if armType == "Left":
            robot.setRestPoses(initialConfig, robot.rightArmCurrConfiguration, robot.rightHandCurrConfiguration)
        else:
            robot.setRestPoses(robot.leftArmCurrConfiguration, initialConfig, robot.rightHandCurrConfiguration)

        pos1 = np.array(pose1[0])
        rot1 = p.getEulerFromQuaternion(pose1[1])
        rot1 = np.array(rot1)
        pos2 = np.array(pose2[0])
        rot2 = p.getEulerFromQuaternion(pose2[1])
        rot2 = np.array(rot2)

        config_edge_traj = []
        dx = 0.001
        dtheta = 0.1 * np.pi / 180
        nseg = int(max(abs(pose1[0][0]-pose2[0][0]), 
                    abs(pose1[0][1]-pose2[0][1]), abs(pose1[0][2]-pose2[0][2])) / dx)
        nseg = max(nseg, int(np.ceil(np.abs(rot2-rot1).max() / dtheta)))
        if nseg == 0: nseg += 1
        # print("nseg: " + str(nseg))
        isEdgeValid = False
        for i in range(1, nseg+1):
            interm_pos = utils.interpolatePosition(pose1[0], pose2[0], 1 / nseg * i)
            interm_quat = utils.interpolateQuaternion(pose1[1], pose2[1], 1 / nseg * i)
            interm_pose = [interm_pos, interm_quat]
            ### check this pose
            isPoseValid, singleArmConfig_IK = self.generateConfigBasedOnPose(interm_pose, robot, workspace, armType, motionType)
            if not isPoseValid:
                config_edge_traj = []
                return isEdgeValid, config_edge_traj
            else:
                config_edge_traj.append(singleArmConfig_IK)
                if armType == "Left":
                    robot.setRestPoses(singleArmConfig_IK, robot.rightArmCurrConfiguration, robot.rightHandCurrConfiguration)
                else:
                    robot.setRestPoses(robot.leftArmCurrConfiguration, singleArmConfig_IK, robot.rightHandCurrConfiguration)
                robot.setSingleArmToConfig(singleArmConfig_IK, armType)

        
        ### Reach here because the edge is valid since all poses along the edge are valid
        isEdgeValid = True
        return isEdgeValid, config_edge_traj

    def checkEdgeValidity_cartesianMove_bk(self, initialConfig, pose1, pose2, robot, workspace, armType, motionType):
        ### set the rest pose to initialConfig
        if armType == "Left":
            ee_idx = robot.left_ee_idx
            joint_position = robot.leftArmCurrConfiguration
            robot.setRestPoses(initialConfig, robot.rightArmCurrConfiguration, robot.rightHandCurrConfiguration)
            q = initialConfig + robot.rightArmCurrConfiguration + robot.rightHandCurrConfiguration
        else:
            ee_idx = robot.right_ee_idx
            joint_position = robot.rightArmCurrConfiguration
            robot.setRestPoses(robot.leftArmCurrConfiguration, initialConfig, robot.rightHandCurrConfiguration)
            q = robot.leftArmCurrConfiguration + initialConfig + robot.rightHandCurrConfiguration
        config_edge_traj = []
        min_dist = 0.01

        pos1 = np.array(pose1[0])
        rot1 = p.getEulerFromQuaternion(pose1[1])
        rot1 = np.array(rot1)
        pos2 = np.array(pose2[0])
        rot2 = p.getEulerFromQuaternion(pose2[1])
        rot2 = np.array(rot2)
        dx = 0.001
        dtheta = 0.1 * np.pi / 180
        nseg = int(max(abs(pose1[0][0]-pose2[0][0]), 
                    abs(pose1[0][1]-pose2[0][1]), abs(pose1[0][2]-pose2[0][2])) / dx)
        nseg = max(nseg, int(np.ceil(np.abs(rot2-rot1).max() / dtheta)))
        if nseg == 0: nseg += 1

        d_pos = (pos2-pos1) / nseg
        d_rot = (rot2-rot1) / nseg
        dx = np.concatenate([d_pos, d_rot], 0)
        q = np.array(q)
        # config_edge_traj.append(q)
        qs = [q]
        # print("nseg: " + str(nseg))
        isEdgeValid = False
        ll = robot.ll
        ul = robot.ul
        ll = np.array(ll)
        ul = np.array(ul)
        print("reach here? 2")

        for i in range(0, nseg-1):
            q = qs[-1]
            interm_pos = pos1 + d_pos * i
            interm_rot = rot1 + d_rot * i

            j_pos, j_rot = p.calculateJacobian(
                bodyUniqueId=robot.motomanGEO, linkIndex=ee_idx, localPosition=[0.0, 0.0, 0.0], objPositions=qs[-1].tolist(), 
                objVelocities=[0.0]*20, objAccelerations=[0.0]*20, physicsClientId=self.planningServer)
            j = np.concatenate([j_pos, j_rot], 0)
            print('obtained jacobisan')
            def opt_func(dq):
                return max((q+dq-ul).max(), 0) + max((ll-q-dq).max(), 0)
            cons = ({'type': 'eq', 'fun': lambda dq: j.dot(dq)-dx})
            res = sop.minimize(opt_func, x0=qs[-1], constraints=cons)
            print('problem solved')
            new_q = q + res.x
            print('jacobian: ')
            print(j)
            print('j.dot(dq):')
            print(j.dot(res.x))
            print('dx:')
            print(dx)
            qs.append(new_q)
            if armType == "Left":
                robot.setSingleArmToConfig(new_q.tolist()[0:7], armType)
            else:
                robot.setSingleArmToConfig(new_q.tolist()[7:14], armType)

        
        print("reach here? 3")

        ### Reach here because the edge is valid since all poses along the edge are valid
        isEdgeValid = True
        print(qs)
        return isEdgeValid, qs




    def checkEdgeValidity_DirectConfigPath(self, n1, n2, robot, workspace, armType, motionType):
        ### n1, n2: [q1, ... q7]

        # nseg = 5
        # min_degree = math.pi / 90
        min_degree = math.pi / 90 * 2 ### make it sparsely interpolated to speed up collision check
        # min_degree = math.pi / 90 / 25 ### JUST TEST ###
        nseg = int(max(
            abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]),
            abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6])) / min_degree)
        if nseg == 0: nseg += 1
        print("nseg: " + str(nseg)) ### JUST TEST ### 

        isEdgeValid = False
        for i in range(0, nseg+1):
            # print("idx: ", i)
            interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
            interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
            interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
            interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
            interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
            interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
            interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
            intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
            ### check this pose
            isPoseValid = self.checkIK_AllCollisions(intermNode, robot, workspace, armType, motionType)
            if not isPoseValid:
                return isEdgeValid

        ### Reach here because the edge is valid since all poses along the edge are valid
        isEdgeValid = True
        return isEdgeValid


    def checkEdgeValidity_knownGEO(self, n1, n2, robot, workspace, armType):
        ### Input: n1, n2: node (a list of 7 joint values)
        ### Output: bool value indicates whether the transition from n1 to n2 is valid

        # nseg = 5
        # min_degree = math.pi / 90
        min_degree = math.pi / 90 * 8 ### make it sparsely interpolated to speed up collision check
        nseg = int(max(
            abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]),
            abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6])) / min_degree)
        if nseg == 0: nseg += 1
        # print("nseg: " + str(nseg))

        isEdgeValid = False
        for i in range(0, nseg+1):
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


    def generateTrajectory_DirectConfigPath(self, n1, n2):
        ### This function generates a trajectory based on two configs (which has been proved to be valid transition)
        ### Input: n1, n2: node (a list of 7 joint values)
        ### output: an edge trajectory (config_edge_traj) which includes the endtail but not the head
        ###         format: a list of list(7 joint values)

        config_edge_traj = []

        # nseg = 5
        min_degree = math.pi / 90 * 2 ### want more waypoint to move more naturally
        nseg = int(max(
            abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]),
            abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6])) / min_degree)
        if nseg == 0: nseg += 1
        # print("nseg: " + str(nseg))

        ### we don't include the head (i=0)
        for i in range(1, nseg+1):
            interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
            interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
            interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
            interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
            interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
            interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
            interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
            intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
            config_edge_traj.append(intermNode)

        return config_edge_traj

    
    def generateTrajectory_approachToGrasp(self, preGraspPose, workspace, armType, robot):

        config_edge_traj = []
        min_dist = 0.005
        nseg = int(0.1 / min_dist)
        if nseg == 0: nseg += 1
        print("nseg in approachToGrasp: ", str(nseg))

        ### approach direction is always 10cm towards z-axis
        temp_rot_matrix = p.getMatrixFromQuaternion(preGraspPose[1])
        ### local z-axis of the end effector
        temp_approaching_direction = [temp_rot_matrix[2], temp_rot_matrix[5], temp_rot_matrix[8]]
        if armType == "Left":
            first_joint_index = 0
            ee_idx = robot.left_ee_idx
            db_start_idx = 0
        else:
            first_joint_index = 7
            ee_idx = robot.right_ee_idx
            db_start_idx = 12
        for i in range(1, nseg+1):
            temp_pos = list(np.array(preGraspPose[0]) + min_dist*i*np.array(temp_approaching_direction))
            temp_pose = [temp_pos, preGraspPose[1]] ### the quaternion remains the same as preGraspPose

            config_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                        endEffectorLinkIndex=ee_idx,
                        targetPosition=temp_pose[0],
                        targetOrientation=temp_pose[1],
                        lowerLimits=robot.ll, upperLimits=robot.ul, 
                        jointRanges=robot.jr, restPoses = robot.rp,
                        maxNumIterations=20000, residualThreshold=0.001,
                        physicsClientId=robot.server)


            config_IK = list(config_IK[first_joint_index:first_joint_index+7])
            config_IK, isIKValid = self.AdjustIKBasedOnJointLimit(config_IK, robot, armType)
            isPoseValid = False
            if isIKValid:
                isPoseValid = self.checkPoseIK(
                        config_IK, ee_idx, temp_pose, robot, workspace, armType, motionType="transit")
            if not isPoseValid:
                print('Pybullet failed. Linear IK')
                config_IK = self.linear_inverse_kinematics(armType, temp_pose, robot.rp, robot, start_pose=None)

                isIKValid = False
                isPoseValid = False

                if config_IK is not None:
                    config_IK, isIKValid = self.AdjustIKBasedOnJointLimit(config_IK, robot, armType)
                    if isIKValid:
                        isPoseValid = self.checkPoseIK(
                            config_IK, ee_idx, temp_pose, robot, workspace, armType, motionType="transit")

            if isPoseValid:
                ### Valid IK
                config_edge_traj.append(config_IK)
                ### update the rest pose
                if armType == "Left":
                    robot.setRestPoses(config_IK, robot.rightArmCurrConfiguration, robot.rightHandCurrConfiguration)
                else:
                    robot.setRestPoses(robot.leftArmCurrConfiguration, config_IK, robot.rightHandCurrConfiguration)
            else:
                continue
        return config_edge_traj

        # ### we don't include the head (i=0)
        # for i in range(1, nseg+1):
        #     interm_pos_x = p1[0][0] + (p2[0][0]-p1[0][0]) / nseg * i
        #     interm_pos_y = p1[0][1] + (p2[0][1]-p1[0][2]) / nseg * i
        #     interm_pos_z = p1[0][2] + (p2[0][1]-p1[0][2]) / nseg * i
        #     interm_pose = [[interm_pos_x, interm_pos_y, interm_pos_z], p2[1]]
        #     singleArmConfig_IK = self.inverse_kinematics(armType, interm_pose, robot.rp)
        #     if singleArmConfig_IK is None:
        #         ### the IK is invalid, skip to next waypoint
        #         continue
        #     else:
        #         ### Valid IK
        #         config_edge_traj.append(singleArmConfig_IK)
        #         ### update the rest pose
        #         if armType == "Left":
        #             self.robot.setRestPoses(singleArmConfig_IK, robot.rightArmCurrConfiguration, robot.rightHandCurrConfiguration)
        #         else:
        #             self.robot.setRestPoses(robot.leftArmCurrConfiguration, singleArmConfig_IK, robot.rightHandCurrConfiguration)
        
        return config_edge_traj


    def generateTrajectory_DirectConfigPath_faster(self, n1, n2):
        ### This function generates a trajectory based on two configs (which has been proved to be valid transition)
        ### Input: n1, n2: node (a list of 7 joint values)
        ### output: an edge trajectory (config_edge_traj) which includes the endtail but not the head
        ###         format: a list of list(7 joint values)

        config_edge_traj = []

        # nseg = 5
        min_degree = math.pi / 90 * 8 ### move faster in single joint condition
        nseg = int(max(
            abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]),
            abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6])) / min_degree)
        if nseg == 0: nseg += 1
        # print("nseg: " + str(nseg))

        ### we don't include the head (i=0)
        for i in range(1, nseg+1):
            interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
            interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
            interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
            interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
            interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
            interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
            interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
            intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
            config_edge_traj.append(intermNode)

        return config_edge_traj


    # def checkAndGenerateSolution_DirectConfigPath(self, n1, n2):
    #     ### This function checks an edge coming from the solution
    #     ### and will generate an edge trajectory if the edge is valid
    #     ### so this function has the features from both generateTrajectory_DirectConfigPath and checkEdgeValidity_DirectConfigPath
    #     ### Input: n1, n2: node (a list of 7 joint values)
    #     ### output: an edge trajectory (config_edge_traj) which includes the endtail but not the head
    #     ###         format: a list of list(7 joint values)
    #     ###         isEdgeValid


    def shortestPathPlanning(self, initialConfig, targetConfig, theme, robot, workspace, armType, motionType):
        ### Input: initialConfig, configToPreGraspPose [q1, q2, ..., q7]
        ### Output: traj (format: [edge_config1, edge_config2, ...])
        ###         and each edge_config is a list of [7*1 config]
        ### first prepare the start_goal file

        result_traj = [] ### the output we want to construct

        f = self.writeStartGoal(initialConfig, targetConfig, theme)
        self.connectStartGoalToArmRoadmap(f,
                initialConfig, targetConfig, robot, workspace, armType, motionType)

        ### call the planning algorithm
        executeCommand = "./main_planner" + " " + theme + " " + armType + " " + str(self.nsamples) + " shortestPath"
        # subprocess.call(executeCommand, cwd="/home/rui/Documents/research/motoman_ws/src/pybullet_motoman/src", shell=True)
        rospack = RosPack()
        cwd = os.path.join(rospack.get_path("pybullet_motoman"), 'src')
        subprocess.call(executeCommand, cwd=cwd, shell=True)

        path = self.readPath(theme, armType)
        if path == []:
            ### the plan fails, could not find a solution
            return result_traj ### an empty trajectory

        ############ LET'S DO SMOOTHING #############
        # if len(path) == 3:
        #     ### no need for collision check and smoothing
        #     for i in range(0, len(path)-1):
        #         if i == 0:
        #             config1 = initialConfig
        #             config2 = self.nodes[armType][path[i+1]]
        #         else:
        #             ### i == 1
        #             config1 = self.nodes[armType][path[i]]
        #             config2 = targetConfig
        #         ### get edge trajectory
        #         config_edge_traj = self.generateTrajectory_DirectConfigPath(config1, config2)
        #         result_traj.append(config_edge_traj)
        # else:
            ### len(path) > 3
            ### we need collision check and smoothing
            smoothed_path, isPathValid = self.smoothPath(path, initialConfig, targetConfig, robot, workspace, armType, motionType)
            print("smoothed path: ", smoothed_path)
            if isPathValid == False:
                return result_traj
            ### directly generate trajectory based on the new path
            for i in range(0, len(smoothed_path)-1):
                if i == 0:
                    config1 = initialConfig
                else:
                    config1 = self.nodes[armType][smoothed_path[i]]
                if i == (len(smoothed_path)-2):
                    config2 = targetConfig
                else:
                    config2 = self.nodes[armType][smoothed_path[i+1]]
                ### get edge trajectory
                config_edge_traj = self.generateTrajectory_DirectConfigPath(config1, config2)
                result_traj.append(config_edge_traj)

        return result_traj


    def smoothPath(self, path, initialConfig, targetConfig, robot, workspace, armType, motionType):
        ### This function tries to smooth the given path
        ### output: a smooth path [a list of indexes] and whether the path is valid or not
        smoothed_path = []
        start_idx = 0
        startNode_idx = path[start_idx] ### start
        smoothed_path.append(startNode_idx)
        curr_idx = start_idx + 1
        while (curr_idx < len(path)):
            currNode_idx = path[curr_idx]
            ### check edge validity between start_idx and curr_idx
            if start_idx == 0:
                config1 = initialConfig
            else:
                config1 = self.nodes[armType][startNode_idx]
            if curr_idx == (len(path)-1):
                config2 = targetConfig
            else:
                config2 = self.nodes[armType][currNode_idx]
            ### check the edge
            isEdgeValid = self.checkEdgeValidity_DirectConfigPath(
                            config1, config2, robot, workspace, armType, motionType)
            if isEdgeValid:
                validFromStart_idx = curr_idx
                validNodeFromStart_idx = currNode_idx
                ### move on to the next node
                curr_idx += 1
                continue
            else:
                if (curr_idx - start_idx == 1):
                    print("Abortion\n")
                    return smoothed_path, False
                ### the edge is not valid
                ### add validNodeFromStart_idx to the smoothed_path
                smoothed_path.append(validNodeFromStart_idx)
                ### set validNodeFromStart_idx as the new start
                ### and the curr_idx does not change
                start_idx = validFromStart_idx
                startNode_idx = validNodeFromStart_idx
        smoothed_path.append(validNodeFromStart_idx)
        # smoothed_path.append(path[-1])

        return smoothed_path, True


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

        return path


    def writeStartGoal(self, initialConfig, targetConfig, theme):
        start_goal_file = os.path.join(self.rosPackagePath, "src/") + theme + ".txt"
        f = open(start_goal_file, "w")
        f.write(str(self.nsamples) + " ")
        for e in initialConfig:
            f.write(str(e) + " ")
        f.write("\n")
        f.write(str(self.nsamples+1) + " ")
        for e in targetConfig:
            f.write(str(e) + " ")
        f.write("\n")

        return f


    def connectStartGoalToArmRoadmap(
                        self, f, initialConfig, targetConfig, robot, workspace, armType, motionType):
        ### initialConfig, targetConfig [q1, q2, ..., q7]
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
            utils.calculateNorm2(initialConfig, neighborConfig) for neighborConfig in self.nodes[armType]]
        neighborIndex_to_start, neighborDist_to_start = zip(
                                    *sorted(enumerate(dist_to_start), key=itemgetter(1)))
        neighborIndex_to_start = list(neighborIndex_to_start)
        neighborDist_to_start = list(neighborDist_to_start)

        dist_to_goal = [
            utils.calculateNorm2(targetConfig, neighborConfig) for neighborConfig in self.nodes[armType]]
        neighborIndex_to_goal, neighborDist_to_goal = zip(
                                    *sorted(enumerate(dist_to_goal), key=itemgetter(1)))
        neighborIndex_to_goal = list(neighborIndex_to_goal) 
        neighborDist_to_goal = list(neighborDist_to_goal)

        # max_neighbors = self.num_neighbors
        max_neighbors = 15
        max_candiates_to_consider = self.num_neighbors

        ####### now connect potential neighbors for the start and the goal #######
        ### for start
        # print("for start")        
        neighbors_connected = 0
        for j in range(max_candiates_to_consider):
            ### first check if the query node has already connected to enough neighbors
            if neighbors_connected >= max_neighbors:
                break
            ### otherwise, find the neighbor
            neighbor = self.nodes[armType][neighborIndex_to_start[j]]
            ### check the edge validity
            isEdgeValid = self.checkEdgeValidity_DirectConfigPath(
                        initialConfig, neighbor, robot, workspace, armType, motionType)
            if isEdgeValid:
                f.write(str(start_id) + " " + str(neighborIndex_to_start[j]) + \
                                             " " + str(neighborDist_to_start[j]) + "\n")
                neighbors_connected += 1
        print("Number of neighbors for start node " + str(start_id) + ": " + str(neighbors_connected))

        ### for goal
        # print("for goal")
        neighbors_connected = 0
        for j in range(max_candiates_to_consider):        
            ### first check if the query node has already connected to enough neighbors
            if neighbors_connected >= max_neighbors:
                break
            ### otherwise, find the neighbor
            neighbor = self.nodes[armType][neighborIndex_to_goal[j]]
            ### check the edge validity
            isEdgeValid = self.checkEdgeValidity_DirectConfigPath(
                        neighbor, targetConfig, robot, workspace, armType, motionType)
            if isEdgeValid:
                f.write(str(target_id) + " " + str(neighborIndex_to_goal[j]) + \
                                            " " + str(neighborDist_to_goal[j]) + "\n")            
                neighbors_connected += 1
        print("Number of neighbors for goal node " + str(target_id) + ": " + str(neighbors_connected))

        f.close()




############### the codes below are not used but kept for legacy ##############

# def samplesConnect_cartesian(self, robot, workspace, armType):
#     connectivity = np.zeros((self.nsamples, self.nsamples))
#     tree = spatial.KDTree(self.workspaceNodes[armType]) ### use KD tree to arrange neighbors assignment
#     connectionsFile = self.roadmapFolder + "/connections_" + str(armType) + ".txt"
#     f_connection = open(connectionsFile, "w")
#     ### for each node
#     for i in range(len(self.workspaceNodes[armType])):
#         queryworkspaceNode = self.workspaceNodes[armType][i]
#         knn = tree.query(queryworkspaceNode, k=self.num_neighbors, p=2)
#         queryNode = self.nodes[armType][i]

#         neighbors_connected = 0
#         ### for each potential neighbor
#         for j in range(len(knn[1])):
#             ### first check if this query node has already connected to enough neighbors
#             if neighbors_connected >= self.num_neighbors:
#                 break
#             if knn[1][j] == i:
#                 ### if the neighbor is the query node itself
#                 continue
#             if connectivity[i][knn[1][j]] == 1:
#                 ### the connectivity has been checked before
#                 neighbors_connected += 1
#                 continue
#             ### Otherwise, check the edge validity
#             ### in terms of collision with the robot itself and all known geometries (e.g. table/shelf)
#             ### between the query node and the current neighbor
#             neighbor = self.nodes[armType][knn[1][j]]
#             # print("query node: ", queryNode)
#             # print("neighbor: ", neighbor)
#             # raw_input("check")
#             isEdgeValid = self.checkEdgeValidity_cartesian(queryNode, neighbor, robot, workspace, armType)
#             if isEdgeValid:
#                 ### write this edge information with their costs and labels into the txt file
#                 f_connection.write(str(i) + " " + str(knn[1][j]) + " " + str(knn[0][j]) + "\n")
#                 connectivity[i][knn[1][j]] = 1
#                 connectivity[knn[1][j]][i] = 1
#                 neighbors_connected += 1
#         print("Number of neighbors for current node " + str(i) + ": " + str(neighbors_connected))
#     f_connection.close()


# def checkEdgeValidity_cartesian(self, w1, w2, robot, workspace, armType):
#     if armType == "Left":
#         ee_idx = robot.left_ee_idx
#     else:
#         ee_idx = robot.right_ee_idx
#     # nseg = 5
#     min_dist = 0.01
#     nseg = int(max(
#         abs(w1[0][0]-w2[0][0]), abs(w1[0][1]-w2[0][1]), abs(w1[0][2]-w2[0][2])) / min_dist)
#     if nseg == 0: nseg += 1
#     # print("nseg: " + str(nseg))

#     isEdgeValid = False
#     for i in range(1, nseg):
#         interm_pos = utils.interpolatePosition(w1[0], w2[0], 1 / nseg * i)
#         interm_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
#                                 endEffectorLinkIndex=ee_idx,
#                                 targetPosition=interm_pos,
#                                 lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
#                                 maxNumIterations=2000, residualThreshold=0.0000001,
#                                 physicsClientId=robot.server)
#         if armType == "Left":
#             interm_IK = list(interm_IK[0:7])
#         else:
#             interm_IK = list(interm_IK[7:14])
#         ### Then check if there is collision
#         isValid = self.checkIK_CollisionWithRobotAndKnownGEO(interm_IK, robot, workspace, armType)
#         if isValid == False:
#             return isEdgeValid

#     ### Reach here because the edge pass the collision check
#     isEdgeValid = True
#     return isEdgeValid


# def IKresetForSingleArm(self, robot, armType):
#     if armType == "Left":
#         first_joint_index = 0
#         config_length = len(robot.leftArmCurrConfiguration)
#     else:
#         first_joint_index = 7
#         config_length = len(robot.rightArmCurrConfiguration)

#     ### reset arm configuration
#     resetSingleArmConfiguration = []
#     for i in range(config_length):
#         resetSingleArmConfiguration.append(
#                 random.uniform(robot.ll[first_joint_index + i], robot.ul[first_joint_index + i]))

#     # resetSingleArmConfiguration = [0.0]*len(robot.leftArmCurrConfiguration)
#     robot.setSingleArmToConfig(resetSingleArmConfiguration, armType)


# def checkPoseBasedOnConfig(self, pose, robot, workspace, armType, motionType, checkType):
#     ### This function checks the validity of a pose by checking its corresponding config (IK)
#     ### Input: pose: [[x,y,z],[x,y,z,w]]
#     ###        armType: "Left" or "Right"
#     ###        motionType: "transfer" or "transit" or "others"
#     ###        checkType: "discrete" or "continuous"
#     ### Output: isPoseValid (bool), singleArmConfig_IK (list (7-by-1))
#     if armType == "Left":
#         ee_idx = robot.left_ee_idx
#         first_joint_index = 0
#     else:
#         ee_idx = robot.right_ee_idx
#         first_joint_index = 7

#     config_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
#                             endEffectorLinkIndex=ee_idx,
#                             targetPosition=pose[0],
#                             targetOrientation=pose[1],
#                             lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
#                             maxNumIterations=2000, residualThreshold=0.0000001,
#                             physicsClientId=robot.server)
#     singleArmConfig_IK = list(config_IK[first_joint_index:first_joint_index+7])
#     isPoseValid = self.checkIK(
#                     singleArmConfig_IK, ee_idx, pose, robot, workspace, armType, motionType)


#     trials = 0
#     while (not isPoseValid) and (trials < 5):
#         if checkType == "discrete":
#             ### reset arm configuration
#             self.IKresetForSingleArm(robot, armType)
#         ### try another IK
#         config_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
#                                 endEffectorLinkIndex=ee_idx,
#                                 targetPosition=pose[0],
#                                 targetOrientation=pose[1],
#                                 lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
#                                 maxNumIterations=2000, residualThreshold=0.0000001,
#                                 physicsClientId=robot.server)
#         singleArmConfig_IK = list(config_IK[first_joint_index:first_joint_index+7])
#         isPoseValid = self.checkIK(
#             singleArmConfig_IK, ee_idx, pose, robot, workspace, armType, motionType)
#         if isPoseValid: break
#         ### otherwise
#         trials += 1
#     ### you need to return both the statement whether the pose is valid
#     ### and the valid configuration the pose corresponds to
#     return isPoseValid, singleArmConfig_IK


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
