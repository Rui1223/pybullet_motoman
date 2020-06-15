from __future__ import division
import pybullet as p
import pybullet_data

import time

from CollisionChecker import CollisionChecker

class MotionExecutor(object):
    def __init__(self, server, camera):
        self.executingServer = server
        self.camera = camera


    def executePath(self, path, robot, handType):
        ### path is a list of configs
        for i in range(len(path)-1):
            current_state = path[i]
            next_state = path[i+1]
            self.state_transition(current_state, next_state, robot, handType)


    def state_transition(self, n1, n2, robot, handType):

        if handType == "Left":
            ee_idx = robot.left_ee_idx
        else:
            ee_idx = robot.right_ee_idx

        nseg = 25
        for i in xrange(0, nseg+1):
            # print("i: " + str(i))
            interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
            interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
            interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
            interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
            interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
            interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
            interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
            intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]

            robot.moveSingleArm(intermNode, handType)

            time.sleep(0.05)
        self.camera.takeImage(self.executingServer, False)


    def local_move(self, pose1, pose2, robot, handType):
        
        if handType == "Left":
            ee_idx = robot.left_ee_idx
        else:
            ee_idx = robot.right_ee_idx

        nseg = 5
        for i in xrange(0, nseg+1):
            temp_x = pose1[0] + (pose2[0]-pose1[0]) / nseg * i
            temp_y = pose1[1] + (pose2[1]-pose1[1]) / nseg * i
            temp_z = pose1[2] + (pose2[2]-pose1[2]) / nseg * i
            ### generate the IK for this intermediate pose
            q_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO_e, 
                                    endEffectorLinkIndex=ee_idx, 
                                    targetPosition=[temp_x, temp_y, temp_z], 
                                    targetOrientation=pose1[3:7], 
                                    lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr, 
                                    maxNumIterations=20000, residualThreshold=0.0000001,
                                    physicsClientId=self.executingServer)
            ### Let's assume the IK is always valid in local_move
            robot.moveDualArm(q_IK)
            time.sleep(0.05)
        self.camera.takeImage(self.executingServer, False)


    def attachTheObject(self, robot, objectInHand, handType, localPose):
        if handType == "Left":
            self.leftArmConstrID = p.createConstraint(
                parentBodyUniqueId=robot.motomanGEO_e, parentLinkIndex=robot.left_ee_idx, 
                childBodyUniqueId=objectInHand.m, childLinkIndex=-1,
                jointType=p.JOINT_FIXED, jointAxis=[0,0,0],
                parentFramePosition=localPose[0:3], childFramePosition=[0,0,0],
                parentFrameOrientation=localPose[3:7],
                physicsClientId=self.executingServer
            )
            
        else:
            self.rightArmConstrID = p.createConstraint(
                parentBodyUniqueId=robot.motomanGEO_e, parentLinkIndex=robot.right_ee_idx, 
                childBodyUniqueId=objectInHand.m, childLinkIndex=-1,
                jointType=p.JOINT_FIXED, jointAxis=[0,0,1],
                parentFramePosition=robot.BasePosition, childFramePosition=objectInHand.pos,
                physicsClientId=self.executingServer
            )



    def disattachTheObject(self, handType):
        if handType == "Left":
            p.removeConstraint(userConstraintUniqueId=self.leftArmConstrID, physicsClientId=self.executingServer)
        else:
            p.removeConstraint(userConstraintUniqueId=self.rightArmConstrID, physicsClientId=self.executingServer)

