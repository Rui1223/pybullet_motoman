from __future__ import division
import pybullet as p
import pybullet_data

import time

from CollisionChecker import CollisionChecker

class MotionExecutor(object):
    def __init__(self, server, camera, isObjectInLeftHand, isObjectInRightHand, objectInLeftHand, objectInRightHand):
        self.executingServer = server
        self.camera = camera
        self.isObjectInLeftHand = isObjectInLeftHand
        self.isObjectInRightHand = isObjectInRightHand
        self.objectInLeftHand = objectInLeftHand
        self.objectInRightHand = objectInRightHand


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

            raw_input("control the move")
            robot.moveSingleArm(intermNode, handType)
            if (self.isObjectInLeftHand and handType == "Left") or (self.isObjectInRightHand and handType == "Right"):
                self.updateRealObjectBasedonLocalPose(robot, handType)
            for k in range(5):
                p.stepSimulation(physicsClientId=self.executingServer)
            if i == nseg:
                print("intermNode: " + str(intermNode))

            time.sleep(0.05)



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
            raw_input("control the move")
            if handType == "Left":
                robot.moveSingleArm(list(q_IK[0:7]), "Left")
            else:
                robot.moveSingleArm(list(q_IK[7:14]), "Right")

            if (self.isObjectInLeftHand and handType == "Left") or (self.isObjectInRightHand and handType == "Right"):
                self.updateRealObjectBasedonLocalPose(robot, handType)
            for k in range(5):
                p.stepSimulation(physicsClientId=self.executingServer)
            time.sleep(0.05)


    def updateRealObjectBasedonLocalPose(self, robot, handType):
        # print("Update object!")
        if handType == "Left":
            ee_idx = robot.left_ee_idx
            objectInHand = self.objectInLeftHand
            localPose = self.localPoseLeft
        else:
            ee_idx = robot.right_ee_idx
            objectInHand = self.objectInRightHand
            localPose = self.localPoseRight
        ls = p.getLinkState(robot.motomanGEO_e, ee_idx, physicsClientId=self.executingServer)
        ee_global_pose = list(ls[0]) + list(ls[1])
        object_global_pose = self.getObjectGlobalPose(localPose, ee_global_pose)
        p.resetBasePositionAndOrientation(
            objectInHand.m, object_global_pose[0:3], object_global_pose[3:7], physicsClientId=self.executingServer)



    def attachTheObject(self, robot, objectInHand, handType, localPose):

        localPoseForObject = p.invertTransform(localPose[0:3], localPose[3:7])

        if handType == "Left":
            self.objectInLeftHand = objectInHand
            self.isObjectInLeftHand = True
            self.localPoseForLeftObject = localPoseForObject
            self.localPoseLeft = localPose
            # self.leftArmConstrID = p.createConstraint(
            #     parentBodyUniqueId=robot.motomanGEO_e, parentLinkIndex=robot.left_ee_idx, 
            #     childBodyUniqueId=objectInHand.m, childLinkIndex=-1,
            #     jointType=p.JOINT_FIXED, jointAxis=[0,0,0],
            #     parentFramePosition=[0,0,0.06], 
            #     childFramePosition=list(self.localPoseForLeftObject[0]),
            #     childFrameOrientation=list(self.localPoseForLeftObject[1]),
            #     physicsClientId=self.executingServer
            # )
            
        else:
            self.objectInRightHand = objectInHand
            self.isObjectInRightHand = True
            self.localPoseForRightObject = localPoseForObject
            self.localPoseRight = localPose
            # self.rightArmConstrID = p.createConstraint(
            #     parentBodyUniqueId=robot.motomanGEO_e, parentLinkIndex=robot.right_ee_idx, 
            #     childBodyUniqueId=objectInHand.m, childLinkIndex=-1,
            #     jointType=p.JOINT_FIXED, jointAxis=[0,0,0],
            #     parentFramePosition=[0,0,0.06],
            #     childFramePosition=list(self.localPoseForRightObject[0]),
            #     childFrameOrientation=list(self.localPoseForRightObject[1]),
            #     physicsClientId=self.executingServer
            # )
        # p.stepSimulation(physicsClientId=self.executingServer)


    def disattachTheObject(self, handType):
        if handType == "Left":
            self.objectInLeftHand = None
            self.isObjectInLeftHand = False
            self.localPoseForLeftObject = None
            self.localPoseLeft = None
            # p.removeConstraint(userConstraintUniqueId=self.leftArmConstrID, physicsClientId=self.executingServer)
        else:
            self.objectInRightHand = None
            self.isObjectInRightHand = False
            self.localPoseForRightObject = None
            self.localPoseRight = None
            # p.removeConstraint(userConstraintUniqueId=self.rightArmConstrID, physicsClientId=self.executingServer)

        # p.stepSimulation(physicsClientId=self.executingServer)



    def getObjectGlobalPose(self, local_pose, ee_global_pose):
        local_pose1 = p.invertTransform(local_pose[0:3], local_pose[3:7])
        
        temp_object_global_pose = p.multiplyTransforms(
            ee_global_pose[0:3], ee_global_pose[3:7],
            list(local_pose1[0]), list(local_pose1[1])
        )

        object_global_pose = list(temp_object_global_pose[0]) + list(temp_object_global_pose[1])

        return object_global_pose

        

