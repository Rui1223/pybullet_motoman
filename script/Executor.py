from __future__ import division
import pybullet as p
import pybullet_data

import time
import math
import IPython

import utils
from CollisionChecker import CollisionChecker

import rospy
from geometry_msgs.msg import Pose 
from pybullet_motoman.msg import EEPoses


class Executor(object):

    def __init__(self, server, 
        isObjectInLeftHand=False, isObjectInRightHand=False,
        objectInLeftHand=None, objectInRightHand=None):
        self.server = server
        self.isObjectInLeftHand = isObjectInLeftHand
        self.isObjectInRightHand = isObjectInRightHand
        self.objectInLeftHand = objectInLeftHand
        self.objectInRightHand = objectInRightHand
        self.leftLocalPose = [[-1, -1, -1], [-1, -1, -1, -1]]
        self.rightLocalPose = [[-1, -1, -1], [-1, -1, -1, -1]]


    def attachObject(self, workspace, robot, armType):
        if armType == "Left":
            self.isObjectInLeftHand = True
            self.objectInLeftHand = workspace.object_geometries.items()[0] ### mesh: name
        else:
            self.isObjectInRightHand = True
            self.objectInRightHand = workspace.object_geometries.items()[0] ### mesh: name

        ### get the object pose relative to the frame of the end effector 
        ### since once the object is attached to the end effector, 
        ### it will remain constant (may add some noice)
        if armType == "Left":
            curr_ee_pose = robot.left_ee_pose
            ls = p.getBasePositionAndOrientation(
                    self.objectInLeftHand[0], physicsClientId=self.server)
            curr_object_global_pose = [list(ls[0]), list(ls[1])]
        else:
            curr_ee_pose = robot.right_ee_pose
            ls = p.getBasePositionAndOrientation(
                    self.objectInRightHand[0], physicsClientId=self.server)
            curr_object_global_pose = [list(ls[0]), list(ls[1])]

        inverse_ee_global = p.invertTransform(curr_ee_pose[0], curr_ee_pose[1])
        temp_localPose = p.multiplyTransforms(
            list(inverse_ee_global[0]), list(inverse_ee_global[1]),
            curr_object_global_pose[0], curr_object_global_pose[1])
        temp_localPose = [list(temp_localPose[0]), list(temp_localPose[1])]

        if armType == "Left":
            self.leftLocalPose = temp_localPose
        else:
            self.rightLocalPose = temp_localPose

        ##################################################################################


    def detachObject(self, workspace, robot, armType):
        if armType == "Left":
            self.isObjectInLeftHand = False
            self.objectInLeftHand = None
        else:
            self.isObjectInRightHand = False
            self.objectInRightHand = None


    def getObjectGlobalPose(self, local_pose, ee_global_pose):
        temp_object_global_pose = p.multiplyTransforms(
            ee_global_pose[0], ee_global_pose[1],
            local_pose[0], local_pose[1])
        object_global_pose = [list(temp_object_global_pose[0]), list(temp_object_global_pose[1])]

        return object_global_pose


    def updateRealObjectBasedonLocalPose(self, robot, armType):
        if armType == "Left":
            ee_idx = robot.left_ee_idx
            objectInHand = self.objectInLeftHand[0]
            curr_ee_pose = robot.left_ee_pose
            object_global_pose = self.getObjectGlobalPose(self.leftLocalPose, curr_ee_pose)

        else:
            ee_idx = robot.right_ee_idx
            objectInHand = self.objectInRightHand[0]
            curr_ee_pose = robot.right_ee_pose
            object_global_pose = self.getObjectGlobalPose(self.rightLocalPose, curr_ee_pose)

        p.resetBasePositionAndOrientation(
            objectInHand, object_global_pose[0], object_global_pose[1], physicsClientId=self.server)

        
    def executePath_cartesian(self, poses_path, robot, armType):
        ### path is a list of poses
        for i in range(len(poses_path)-1):
            current_pose = poses_path[i]
            next_pose = poses_path[i+1]
            self.pose_transition(current_pose, next_pose, robot, armType)
            time.sleep(0.05)

        ### final adjustment
        if armType == "Left":
            ee_idx = robot.left_ee_idx
        else:
            ee_idx = robot.right_ee_idx
        pose_quat = p.getLinkState(robot.motomanGEO, ee_idx, physicsClientId=robot.server)
        curr_pose = [list(pose_quat[0]), list(pose_quat[1])]

        self.final_adjustment(curr_pose, poses_path[-1], robot, armType)

        # curr_config = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
        #                         endEffectorLinkIndex=ee_idx,
        #                         targetPosition=pose[0:3],
        #                         targetOrientation=pose[3:7],
        #                         lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
        #                         maxNumIterations=20000, residualThreshold=0.0000001,
        #                         physicsClientId=robot.server)
        # if armType == "Left":
        #     curr_config = curr_config[0:7]
        # else:
        #     curr_config = curr_config[7:14]
        # self.state_transition(curr_config, targetConfig, robot, armType)


    def executePath(self, path, robot, armType):
        ### path is a list of configs
        for i in range(len(path)-1):
            current_state = path[i]
            next_state = path[i+1]
            self.state_transition(current_state, next_state, robot, armType)
            time.sleep(0.05)


    def final_adjustment(self, w1, w2, robot, armType):
        nseg = 20
        if armType == "Left":
            ee_idx = robot.left_ee_idx
        else:
            ee_idx = robot.right_ee_idx
        # min_dist = 0.05
        # nseg = int(max(
        #     abs(w1[0]-w2[0]), abs(w1[1]-w2[1]), abs(w1[2]-w2[2])) / min_dist)
        # if nseg == 0: nseg += 1
        # print("nseg: " + str(nseg))

        for i in range(1, nseg+1):
            interm_pos = utils.interpolatePosition(w1[0], w2[0], 1 / nseg * i)
            interm_quat = utils.interpolateQuaternion(w1[1], w2[1], 1 / nseg *i)
            interm_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                    endEffectorLinkIndex=ee_idx,
                                    targetPosition=interm_pos,
                                    targetOrientation=interm_quat,
                                    lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                                    maxNumIterations=20000, residualThreshold=0.0000001,
                                    physicsClientId=robot.server)
            if armType == "Left": 
                interm_IK = interm_IK[0:7]
            else:
                interm_IK = interm_IK[7:14]
            robot.moveSingleArm(interm_IK, armType)
            if (self.isObjectInLeftHand and armType == "Left") or (self.isObjectInRightHand and armType == "Right"):
                self.updateRealObjectBasedonLocalPose(robot, armType)

            # p.stepSimulation(physicsClientId=self.server)
            time.sleep(0.005)


    def executeTrajctory(self, trajectory, robot, armType):
        for edge_configs in trajectory:
            for config in edge_configs:
                robot.moveSingleArm(config, armType)
                if (self.isObjectInLeftHand and armType == "Left") or (self.isObjectInRightHand and armType == "Right"):
                    self.updateRealObjectBasedonLocalPose(robot, armType)
                time.sleep(0.05)
            time.sleep(0.05)

    def executeMoveItTrajctory(self, trajectory, robot, armType):
        for config in trajectory:
            robot.moveSingleArm(config, armType)
            if (self.isObjectInLeftHand and armType == "Left") or (self.isObjectInRightHand and armType == "Right"):
                self.updateRealObjectBasedonLocalPose(robot, armType)
            time.sleep(0.1)


    def pose_transition(self, w1, w2, robot, armType):
        # nseg = 5
        if armType == "Left":
            ee_idx = robot.left_ee_idx
        else:
            ee_idx = robot.right_ee_idx
        min_dist = 0.01
        nseg = int(max(
            abs(w1[0][0]-w2[0][0]), abs(w1[0][1]-w2[0][1]), abs(w1[0][2]-w2[0][2])) / min_dist)
        if nseg == 0: nseg += 1
        # print("nseg: " + str(nseg))

        for i in range(1, nseg+1):
            interm_w0 = w1[0][0] + (w2[0][0]-w1[0][0]) / nseg * i
            interm_w1 = w1[0][1] + (w2[0][1]-w1[0][1]) / nseg * i
            interm_w2 = w1[0][2] + (w2[0][2]-w1[0][2]) / nseg * i
            interm_pos = [interm_w0, interm_w1, interm_w2]
            # interm_quat = utils.interpolateQuaternion(w1[3:7], w2[3:7], 1 / nseg *i)
            interm_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                    endEffectorLinkIndex=ee_idx,
                                    targetPosition=interm_pos,
                                    lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                                    maxNumIterations=20000, residualThreshold=0.0000001,
                                    physicsClientId=robot.server)
            if armType == "Left": 
                interm_IK = interm_IK[0:7]
            else:
                interm_IK = interm_IK[7:14]
            robot.moveSingleArm(interm_IK, armType)
            # print("isObjectInRightHand: ", self.isObjectInRightHand)
            if (self.isObjectInLeftHand and armType == "Left") or (self.isObjectInRightHand and armType == "Right"):
                self.updateRealObjectBasedonLocalPose(robot, armType)
            # p.stepSimulation(physicsClientId=self.server)
            time.sleep(0.05)


    def state_transition(self, n1, n2, robot, armType):

        # nseg = 25
        min_degree = math.pi / 90
        nseg = int(max(
            abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]), 
            abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6])) / min_degree)
        if nseg == 0: nseg += 1
        print("nseg: " + str(nseg))

        for i in range(1, nseg+1):
            interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
            interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
            interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
            interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
            interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
            interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
            interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
            intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]

            robot.moveSingleArm(intermNode, armType)
            # curr_waypoint = utils.convertRobotConfig_singleArm(intermNode, robot, armType, self.server)
            # self.traj.append(curr_waypoint)
            # if (self.isObjectInLeftHand and armType == "Left") or (self.isObjectInRightHand and armType == "Right"):
            #     self.updateRealObjectBasedonLocalPose(robot, armType)

            # p.stepSimulation(physicsClientId=self.server)
            time.sleep(0.05)


    def prepare_ee_poses_msgs(self, robot):
        left_ee_position, left_ee_orientation = self.get_ee_pos(robot, "Left")
        right_ee_position, right_ee_orientation = self.get_ee_pos(robot, "Right")
        ee_poses_msgs = EEPoses()
        ee_poses_msgs.left_ee_pose.position.x = left_ee_position[0]
        ee_poses_msgs.left_ee_pose.position.y = left_ee_position[1]
        ee_poses_msgs.left_ee_pose.position.z = left_ee_position[2]
        ee_poses_msgs.left_ee_pose.orientation.x = left_ee_orientation[0]
        ee_poses_msgs.left_ee_pose.orientation.y = left_ee_orientation[1]
        ee_poses_msgs.left_ee_pose.orientation.z = left_ee_orientation[2]
        ee_poses_msgs.left_ee_pose.orientation.w = left_ee_orientation[3]
        ee_poses_msgs.right_ee_pose.position.x = right_ee_position[0]
        ee_poses_msgs.right_ee_pose.position.y = right_ee_position[1]
        ee_poses_msgs.right_ee_pose.position.z = right_ee_position[2]
        ee_poses_msgs.right_ee_pose.orientation.x = right_ee_orientation[0]
        ee_poses_msgs.right_ee_pose.orientation.y = right_ee_orientation[1]
        ee_poses_msgs.right_ee_pose.orientation.z = right_ee_orientation[2]
        ee_poses_msgs.right_ee_pose.orientation.w = right_ee_orientation[3]

        ee_poses_msgs.isObjectInLeftHand = self.isObjectInLeftHand
        ee_poses_msgs.isObjectInRightHand = self.isObjectInRightHand

        return ee_poses_msgs


    def get_ee_pos(self, robot, armType):
        if armType == "Left":
            ee_idx = robot.left_ee_idx
        else:
            ee_idx = robot.right_ee_idx
        ls = p.getLinkState(robot.motomanGEO, ee_idx, physicsClientId=self.server)

        return list(ls[0]), list(ls[1])


