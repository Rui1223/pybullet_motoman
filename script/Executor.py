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
        isObjectInLeftHand, isObjectInRightHand,
        objectInLeftHand, objectInRightHand):
        self.executingServer = server
        self.isObjectInLeftHand = isObjectInLeftHand
        self.isObjectInRightHand = isObjectInRightHand
        self.objectInLeftHand = objectInLeftHand
        self.objectInRightHand = objectInRightHand


    def executePath(self, path, robot, armType):
        ### path is a list of configs
        for i in range(len(path)-1):
            current_state = path[i]
            next_state = path[i+1]
            self.state_transition(current_state, next_state, robot, armType)


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
            # curr_waypoint = utils.convertRobotConfig_singleArm(intermNode, robot, armType, self.executingServer)
            # self.traj.append(curr_waypoint)
            # if (self.isObjectInLeftHand and armType == "Left") or (self.isObjectInRightHand and armType == "Right"):
            #     self.updateRealObjectBasedonLocalPose(robot, armType)

            p.stepSimulation(physicsClientId=self.executingServer)
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
        ls = p.getLinkState(robot.motomanGEO, ee_idx, physicsClientId=self.executingServer)

        return list(ls[0]), list(ls[1]) 

