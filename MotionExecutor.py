from __future__ import division
import pybullet as p
import pybullet_data

import time

from CollisionChecker import CollisionChecker

class MotionExecutor(object):
    def __init__(self, server):
        self.executingServer = server


    def executePath(self, path, robot, handType, robotGEO):
        ### path is a list of configs
        for i in range(len(path)-1):
            current_state = path[i]
            next_state = path[i+1]
            self.local_move(current_state, next_state, robot, handType, robotGEO)


    def local_move(self, n1, n2, robot, handType, robotGEO):

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
            robot.moveSingleArm(intermNode, robotGEO, handType, self.executingServer)

            time.sleep(0.05)


            # if objectInHand == None:
            #     pass
            # else:
            #     ### get the real-time end-effector state
            #     ls = p.getLinkState(robot.motomanGEO_e, robot.left_ee_idx, physicsClientId=self.executingServer)
            #     ee_global_pose = list(ls[0]) + list(ls[1])
            #     # print("ee_global_pose: " + str(ee_global_pose))
            #     object_global_pose = self.getObjectGlobalPose(localPose, ee_global_pose)
            #     objectInHand = self.updateGraspedObject(object_global_pose, objectInHand, self.executingServer, objectInfos)



    def attachTheObject(self, robot, objectInHand, handType, ee_global_pos):
        if handType == "Left":
            self.leftArmConstrID = p.createConstraint(
                parentBodyUniqueId=robot.motomanGEO_e, parentLinkIndex=robot.left_ee_idx, 
                childBodyUniqueId=objectInHand.m, childLinkIndex=-1,
                jointType=p.JOINT_FIXED, jointAxis=[0,0,0],
                parentFramePosition=[0,0,0], childFramePosition=[0,0,0],
                physicsClientId=self.executingServer
            )
            p.changeConstraint(userConstraintUniqueId=self.leftArmConstrID, maxForce=300)
            
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


    ############### the following code is not used but kept here for legacy ######################
    # def getObjectGlobalPose(self, local_pose, ee_global_pose):
    #     local_pose1 = p.invertTransform(local_pose[0:3], local_pose[3:7])
        
    #     temp_object_global_pose = p.multiplyTransforms(
    #         ee_global_pose[0:3], ee_global_pose[3:7],
    #         list(local_pose1[0]), list(local_pose1[1])
    #     )

    #     object_global_pose = list(temp_object_global_pose[0]) + list(temp_object_global_pose[1])

    #     return object_global_pose

    # def updateGraspedObject(self, object_global_pose, objectInHand, clientID, objectInfos):
    #     ### first delete current mesh of the grasped object
    #     self.removeOldMesh(objectInHand, clientID)
    #     mesh_object = self.addNewMesh(objectInHand, object_global_pose, clientID, objectInfos)
    #     return mesh_object

    # def addNewMesh(self, mesh_object, object_global_pose, clientID, objectInfos):
    #     _c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
    #             fileName=objectInfos[mesh_object.objName][0], meshScale=[1, 1, 1], physicsClientId=clientID)
    #     _v = p.createVisualShape(shapeType=p.GEOM_MESH, 
    #             fileName=objectInfos[mesh_object.objName][0], meshScale=[1, 1, 1], physicsClientId=clientID)
    #     pos = object_global_pose[0:3]
    #     quat = object_global_pose[3:7]
    #     angles = list(p.getEulerFromQuaternion(quat))
    #     _m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
    #                                 basePosition=pos, baseOrientation=quat, physicsClientId=clientID)
    #     mesh_object.update_m(_m)
    #     mesh_object.update_pos(pos)
    #     mesh_object.update_quat(quat)
    #     mesh_object.update_angles(angles)

    #     return mesh_object

    # def removeOldMesh(self, mesh_object, clientID):
    #     p.removeBody(mesh_object.m, clientID)
