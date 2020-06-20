from __future__ import division
import pybullet as p
import pybullet_data

import pybullet_utils.bullet_client as bc
import pybullet_utils.urdfEditor as ed

from collections import OrderedDict
import math
import numpy as np
import os
import IPython
import time

from MotomanRobot import MotomanRobot
from Workspace import Workspace
from Camera import AzureKineticCamera
from MotionPlanner import MotionPlanner
from MotionExecutor import MotionExecutor

class Experiment(object):
    ### This Experiment class defines how a experiment is performed
    def __init__(self, scene_index, exp_mode, saveImages):
        self.scene_index = scene_index
        self.exp_mode = exp_mode
        self.saveImages = saveImages
        ### get two servers, planning and execution respectively
        self.servers = self.genServers(exp_mode)
        p.setGravity(0.0, 0.0, 0.0, physicsClientId=self.servers[1])
        # p.setRealTimeSimulation(enableRealTimeSimulation=1, physicsClientId=self.servers[1])
        ### set the robot ready
        self.robot = MotomanRobot(self.servers)
        ### set the workspace (table, shelf, or whatever specified)
        self.workspace = Workspace("Table", self.robot.BasePosition, self.servers)
        ### get all the geometries from the robot and the workspace
        self.planningGeometries = self.robot.known_geometries_planning + self.workspace.known_geometries_planning
        self.executingGeometries = self.robot.known_geometries_executing + self.workspace.known_geometries_executing
        ### set the camera ready
        self.camera = AzureKineticCamera(self.scene_index, self.workspace.tablePosition, self.workspace.table_dim)

        ### Let's generate a roadmap for any future planner to work on
        startTime = time.clock()
        self.planner = MotionPlanner(self.servers[0], self.scene_index, self.camera)
        self.executor = MotionExecutor(self.servers[1], self.camera, False, False, None, None)
        print("Time spent for loading the nodes: " + str(time.clock() - startTime) + "\n")

        ####### specify some key locations of the arms (placement, handoff, recover) #######
        ### specify the pose for placing the object for the left arm
        self.vacuumPlacementPose = \
            self.workspace.objectDropCenter + list(p.getQuaternionFromEuler([0.0, math.pi, 0.0]))
        self.vacuumPlacementConfig = self.convertPoseToConfig(self.vacuumPlacementPose, "Left")
        ### specify the pose for handoff the object for the left arm
        self.vacuumHandoffPose = \
            self.workspace.objectHandOffCenter + list(p.getQuaternionFromEuler([0.0, math.pi, 0.0]))
        self.vacuumHandoffConfig = self.convertPoseToConfig(self.vacuumHandoffPose, "Left")
        ### specify the pose after finish the task for the left arm
        self.vacuumFinishPose = \
            self.workspace.leftArmFinishCenter + list(p.getQuaternionFromEuler([math.pi/2, math.pi/2, 0.0]))
        self.vacuumFinishConfig = self.convertPoseToConfig(self.vacuumFinishPose, "Left")
        ### specify the pose for placing the object for the right arm
        self.fingerPlacementPose = \
            self.workspace.objectDropCenter + list(p.getQuaternionFromEuler([0.0, math.pi, 0.0]))
        self.fingerPlacementConfig = self.convertPoseToConfig(self.fingerPlacementPose, "Right")
        print("fingerPlacementConfig: " + str(self.fingerPlacementConfig))
        ######################################################################################################



    def runExperiment(self):
        ### get objects information from a txt file
        self.objectsInfos, self.targetObjectName = self.getObjectsInfos(self.scene_index)
        self.objectsPoses = self.loadObjects_p(self.servers[0])
        self.realObjsPoses = self.loadObjects_e(self.servers[1])
        self.objectInHand_p = self.objectsPoses[0]
        self.objectInHand_e = self.realObjsPoses[0]
        
        ### Take the initial image
        self.camera.takeImage(self.servers[0], self.saveImages)

        ### Now Let's check valid picking poses for vacuum gripper
        ####################################################################################################
        self.vacuum_pose_file = os.path.join(os.getcwd(), "poses", self.scene_index, "vacuumPickPoses.txt")
        self.vacuumPickPoses, self.vacuumPickConfigs, self.vacuumLocalPoses, \
                self.vacuumPrePickPoses, self.vacuumPrePickConfigs = self.getVacuumPickPoses(self.objectInHand_p)

        self.vacuumPrePickPaths = []
        for i in range(len(self.vacuumPrePickConfigs)):
            temp_path = self.planner.shortestPathPlanning(
                    self.robot.leftArmCurrConfiguration_p, self.vacuumPrePickConfigs[i], "LeftPrePick_"+str(i), 
                    self.robot, self.workspace, "Left")
            self.vacuumPrePickPaths.append(temp_path)
        for i in range(len(self.vacuumPrePickPaths)):
            if len(self.vacuumPrePickPaths[i]) == 0:
                self.vacuumPrePickPaths.remove(self.vacuumPrePickPaths[i])
                self.vacuumPickPoses.remove(self.vacuumPickPoses[i])
                self.vacuumPickConfigs.remove(self.vacuumPickConfigs[i])
                self.vacuumLocalPoses.remove(self.vacuumLocalPoses[i])
                self.vacuumPrePickPoses.remove(self.vacuumPrePickPoses[i])
                self.vacuumPrePickConfigs.remove(self.vacuumPrePickConfigs[i])
        
        
        ####################################################################################################
        '''
        ### for the first local pick
        ### let's demonstrate the left arm successful picking task
        leftPick_id = 0
        ### planning the path for the left arm from picking to placing
        self.vacuumPlacementPath = []
        # print("vacuumPlacementConfig: " + str(self.vacuumPlacementConfig))
        temp_path = self.planner.shortestPathPlanning(
                self.vacuumPickConfigs[leftPick_id], self.vacuumPlacementConfig, "LeftPlace_"+str(leftPick_id),
                self.robot, self.workspace, "Left")
        self.vacuumPlacementPath.append(temp_path)
        ### planning the path for the left arm from placing to finishing
        self.vacuumFinishPath = []
        temp_path = self.planner.shortestPathPlanning(
            self.vacuumPlacementConfig, self.vacuumFinishConfig, "LeftFinish_"+str(0), self.robot, self.workspace, "Left")
        self.vacuumFinishPath.append(temp_path)

        raw_input("enter to continue")
        ### Let's execute the path for this task
        self.executor.executePath(self.vacuumPrePickPaths[leftPick_id], self.robot, "Left")
        self.executor.executePath([self.vacuumPrePickConfigs[leftPick_id], self.vacuumPickConfigs[leftPick_id]], self.robot, "Left")
        # self.executor.local_move(
        #     self.vacuumPrePickPoses[leftPick_id], self.vacuumPickPoses[leftPick_id], self.robot, "Left")

        self.executor.attachTheObject(self.robot, self.objectInHand_e, "Left", self.vacuumLocalPoses[leftPick_id])
        self.executor.executePath(self.vacuumPlacementPath[0], self.robot, "Left")
        time.sleep(1)
        self.executor.disattachTheObject("Left")
        time.sleep(1)
        self.executor.executePath(self.vacuumFinishPath[0], self.robot, "Left")
        '''

        ### for the second local pick
        ### let's demonstrate the left arm handoff to righ arm and right arm place the object
        self.finger_pose_file = os.path.join(os.getcwd(), "poses", self.scene_index, "fingerPickPoses.txt")
        leftPick_id = 1
        ### planning the path for the left arm from picking to handoff
        self.vacuumHandoffPath = []
        temp_path = self.planner.shortestPathPlanning(
            self.vacuumPickConfigs[leftPick_id], self.vacuumHandoffConfig, "LeftHandOff_"+str(0), self.robot, self.workspace, "Left")
        self.vacuumHandoffPath.append(temp_path)
        ### planning the path for the left arm from placing to finishing
        self.vacuumFinishPath = []
        temp_path = self.planner.shortestPathPlanning(
            self.vacuumHandoffConfig, self.vacuumFinishConfig, "LeftFinish_"+str(0), self.robot, self.workspace, "Left")
        self.vacuumFinishPath.append(temp_path)


        ### set both the arm and the object to that config
        self.robot.setSingleArmToConfig(self.vacuumHandoffConfig, "Left")
        self.robot.updateLeftArmConfig(self.vacuumHandoffConfig, self.servers[0])
        self.updatePlanningObjectBasedonLocalPose(self.objectInHand_p, self.vacuumLocalPoses[leftPick_id], "Left")


        #### Let's filter out invalid finger pick poses
        self.fingerPickPoses, self.fingerPickConfigs, self.fingerLocalPoses, \
                self.fingerPrePickPoses, self.fingerPrePickConfigs = self.getFingerPickPoses(self.objectInHand_p)
        
        self.fingerPrePickPaths = []
        for i in range(len(self.fingerPrePickConfigs)):
            temp_path = self.planner.shortestPathPlanning(
                    self.robot.rightArmCurrConfiguration_p, self.fingerPrePickConfigs[i], "rightPrePick_"+str(i), 
                    self.robot, self.workspace, "Right")
            self.fingerPrePickPaths.append(temp_path)
        for i in range(len(self.fingerPrePickPaths)):
            if len(self.fingerPrePickPaths[i]) == 0:
                self.fingerPrePickPaths.remove(self.fingerPrePickPaths[i])
                self.fingerPickPoses.remove(self.fingerPickPoses[i])
                self.fingerPickConfigs.remove(self.fingerPickConfigs[i])
                self.fingerLocalPoses.remove(self.fingerLocalPoses[i])
                self.fingerPrePickPoses.remove(self.fingerPrePickPoses[i])
                self.fingerPrePickConfigs.remove(self.fingerPrePickConfigs[i])


        ### planning the path for the right arm from picking to placement
        self.fingerPlacementPath = []
        temp_path = self.planner.shortestPathPlanning(
            self.fingerPrePickConfigs[0], self.fingerPlacementConfig, "RightPlacem_"+str(0), self.robot, self.workspace, "Right")
        self.fingerPlacementPath.append(temp_path)
        

        
        ### Let's execute the path for this task
        # raw_input("enter to continue")
        self.executor.executePath(self.vacuumPrePickPaths[leftPick_id], self.robot, "Left")
        raw_input("reach pre-grasp")
        self.executor.executePath([self.vacuumPrePickConfigs[leftPick_id], self.vacuumPickConfigs[leftPick_id]], self.robot, "Left")
        raw_input("reach grasp")
        print("vacuumPickConfig: " + str(self.vacuumPickConfigs[leftPick_id]))
        # self.executor.local_move(
        #     self.vacuumPrePickPoses[leftPick_id], self.vacuumPickPoses[leftPick_id], self.robot, "Left")
        raw_input("try to attach")
        self.executor.attachTheObject(self.robot, self.objectInHand_e, "Left", self.vacuumLocalPoses[leftPick_id])

        raw_input("attached!")

        self.executor.executePath(self.vacuumHandoffPath[0], self.robot, "Left")
        # time.sleep(1)

        self.executor.executePath(self.fingerPrePickPaths[0], self.robot, "Right")

        
        self.executor.local_move(
            self.fingerPrePickPoses[0], self.fingerPickPoses[0], self.robot, "Right")
        # time.sleep(1)
        self.executor.attachTheObject(self.robot, self.objectInHand_e, "Right", self.fingerLocalPoses[0])
        # time.sleep(1)
        self.executor.disattachTheObject("Left")
        self.executor.executePath(self.vacuumFinishPath[0], self.robot, "Left")
        # time.sleep(1)
        self.executor.executePath(self.fingerPlacementPath[0], self.robot, "Right")   
        # time.sleep(1)
        self.executor.disattachTheObject("Right")

        time.sleep(1)
        # p.setGravity(0.0, 0.0, -9.8, physicsClientId=self.servers[1])
        p.setRealTimeSimulation(enableRealTimeSimulation=1, physicsClientId=self.servers[1])

        

        
      

    def updatePlanningObjectBasedOnRealScene(self):

        ls = p.getBasePositionAndOrientation(self.objectInHand_e.m, physicsClientId=self.servers[1])
        print("ls: " + str(ls))
        object_global_pose = list(ls[0]) + list(ls[1])
        ### remove the old mesh of the grasped objectg
        self.removeOldMesh(self.objectInHand_p, self.servers[0])
        self.addNewMesh(self.objectInHand_p, object_global_pose, self.servers[0])        

    def updatePlanningObjectBasedonLocalPose(self, objectInhand, localPose, handType):
        if handType == "Left":
            ee_idx = self.robot.left_ee_idx
        else:
            ee_idx = self.robot.right_ee_idx
        ls = p.getLinkState(self.robot.motomanGEO_p, ee_idx, physicsClientId=self.servers[0])
        ee_global_pose = list(ls[0]) + list(ls[1])
        object_global_pose = self.getObjectGlobalPose(localPose, ee_global_pose)
        ### remove the old mesh of the grasped object
        self.removeOldMesh(objectInhand, self.servers[0])
        self.addNewMesh(objectInhand, object_global_pose, self.servers[0])

    def updateRealObjectBasedonLocalPose(self, objectInhand, localPose, handType):

        if handType == "Left":
            ee_idx = self.robot.left_ee_idx
        else:
            ee_idx = self.robot.right_ee_idx   
        ls = p.getLinkState(self.robot.motomanGEO_e, ee_idx, physicsClientId=self.servers[1])
        ee_global_pose = list(ls[0]) + list(ls[1])
        object_global_pose = self.getObjectGlobalPose(localPose, ee_global_pose)
        p.resetBasePositionAndOrientation(
            objectInhand.m, object_global_pose[0:3], object_global_pose[3:7], physicsClientId=self.servers[1])
        p.stepSimulation(physicsClientId=self.servers[1])


    def addNewMesh(self, mesh_object, object_global_pose, clientID):
        _c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
                fileName=self.objectsInfos[mesh_object.objName][0], meshScale=[1, 1, 1], physicsClientId=clientID)
        _v = p.createVisualShape(shapeType=p.GEOM_MESH, 
                fileName=self.objectsInfos[mesh_object.objName][0], meshScale=[1, 1, 1], physicsClientId=clientID)
        pos = object_global_pose[0:3]
        quat = object_global_pose[3:7]
        angles = list(p.getEulerFromQuaternion(quat))
        _m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
                                    basePosition=pos, baseOrientation=quat, physicsClientId=clientID)
        mesh_object.update_m(_m)
        mesh_object.update_pos(pos)
        mesh_object.update_quat(quat)
        mesh_object.update_angles(angles)


    def removeOldMesh(self, object_mesh, clientID):
        p.removeBody(object_mesh.m, clientID)
        

    def getObjectGlobalPose(self, local_pose, ee_global_pose):
        local_pose1 = p.invertTransform(local_pose[0:3], local_pose[3:7])
        
        temp_object_global_pose = p.multiplyTransforms(
            ee_global_pose[0:3], ee_global_pose[3:7],
            list(local_pose1[0]), list(local_pose1[1])
        )

        object_global_pose = list(temp_object_global_pose[0]) + list(temp_object_global_pose[1])

        return object_global_pose


    def convertPoseToConfig(self, armPose, handType):
        if handType == "Left":
            ee_idx = self.robot.left_ee_idx
        else:
            ee_idx = self.robot.right_ee_idx
        ### get the corresponding IK for the currently examined pose
        q_IK = p.calculateInverseKinematics(bodyUniqueId=self.robot.motomanGEO_p, 
                                endEffectorLinkIndex=ee_idx, 
                                targetPosition=armPose[0:3], 
                                targetOrientation=armPose[3:7], 
                                lowerLimits=self.robot.ll, upperLimits=self.robot.ul, jointRanges=self.robot.jr, 
                                maxNumIterations=20000, residualThreshold=0.0000001,
                                physicsClientId=self.servers[0])
        isValid = self.planner.checkIK(
            q_IK, ee_idx, armPose[0:3], self.robot, self.workspace)

        if handType == "Left":
            return list(q_IK[0:7])
        else:
            return list(q_IK[7:14])


    def getVacuumPickPoses(self, targetObj):
        vacuumPickPoses = []
        vacuumPickConfigs = []
        vacuumLocalPoses = []
        vacuumPrePickPoses = []
        vacuumPrePickConfigs = []
        f = open(self.vacuum_pose_file, "r")
        for line in f:
            line = line.split(',')
            temp_pose = [float(temp_e) for temp_e in line]
            ### check this pose ###
            ### step 1: convert it from target object local frame to global frame in the workspace
            global_pose = p.multiplyTransforms(
                targetObj.pos, targetObj.quat,
                temp_pose[0:3], p.getQuaternionFromEuler([i*math.pi/180 for i in temp_pose[3:6]])
            )
            global_pose = list(global_pose[0]) + list(global_pose[1])

            ### step 2: get the corresponding IK for the currently examined pose
            q_graspIK = p.calculateInverseKinematics(bodyUniqueId=self.robot.motomanGEO_p, 
                                    endEffectorLinkIndex=self.robot.left_ee_idx, 
                                    targetPosition=global_pose[0:3], 
                                    targetOrientation=global_pose[3:7], 
                                    lowerLimits=self.robot.ll, upperLimits=self.robot.ul, jointRanges=self.robot.jr, 
                                    maxNumIterations=20000, residualThreshold=0.0000001,
                                    physicsClientId=self.servers[0])


            ### step 3: check the validity of the IK
            isValid = self.planner.checkIK(
                q_graspIK, self.robot.left_ee_idx, global_pose[0:3], self.robot, self.workspace)
            # raw_input("ENTER to continue")

            if isValid:
                ### generate pre-grasp pose and check its validity
                prePickPose, prePickConfig = self.getPrePickPose(global_pose, "Left")
                if prePickConfig != None:
                    ### the pre-grasp is also valid
                    ### Now let's add the poses and configs
                    vacuumPickPoses.append(global_pose)
                    vacuumPickConfigs.append(list(q_graspIK[0:7]))
                    vacuumLocalPoses.append(temp_pose[0:3] + list(p.getQuaternionFromEuler([i*math.pi/180 for i in temp_pose[3:6]])))
                    vacuumPrePickPoses.append(prePickPose)
                    vacuumPrePickConfigs.append(prePickConfig)


        return vacuumPickPoses, vacuumPickConfigs, vacuumLocalPoses, vacuumPrePickPoses, vacuumPrePickConfigs


    def getFingerPickPoses(self, targetObj):
        fingerPickPoses = []
        fingerPickConfigs = []
        fingerLocalPoses = []
        fingerPrePickPoses = []
        fingerPrePickConfigs = []
        f = open(self.finger_pose_file, "r")
        for line in f:
            line = line.split(',')
            temp_pose = [float(temp_e) for temp_e in line]
            ### check this pose ###
            ### step 1: convert it from target object local frame to global frame in the workspace
            global_pose = p.multiplyTransforms(
                targetObj.pos, targetObj.quat,
                temp_pose[0:3], p.getQuaternionFromEuler([i*math.pi/180 for i in temp_pose[3:6]])
            )
            global_pose = list(global_pose[0]) + list(global_pose[1])

            ### step 2: get the corresponding IK for the currently examined pose
            q_graspIK = p.calculateInverseKinematics(bodyUniqueId=self.robot.motomanGEO_p, 
                                    endEffectorLinkIndex=self.robot.right_ee_idx, 
                                    targetPosition=global_pose[0:3], 
                                    targetOrientation=global_pose[3:7], 
                                    lowerLimits=self.robot.ll, upperLimits=self.robot.ul, jointRanges=self.robot.jr, 
                                    maxNumIterations=20000, residualThreshold=0.0000001,
                                    physicsClientId=self.servers[0])

            ### step 3: check the validity of the IK
            isValid = self.planner.checkIK(
                q_graspIK, self.robot.right_ee_idx, global_pose[0:3], self.robot, self.workspace)
            
            if isValid:
                # raw_input("ready to see pre-grasp")
                ### generate pre-grasp pose and check its validity
                prePickPose, prePickConfig = self.getPrePickPose(global_pose, "Right")
                # raw_input("let's see next grasp")
                if prePickConfig != None:
                    ### the pre-grasp is also valid
                    ### Now let's add the poses and configs                    
                    fingerPickPoses.append(global_pose)
                    fingerPickConfigs.append(list(q_graspIK[0:7]))
                    fingerLocalPoses.append(temp_pose[0:3] + list(p.getQuaternionFromEuler([i*math.pi/180 for i in temp_pose[3:6]])))
                    fingerPrePickPoses.append(prePickPose)
                    fingerPrePickConfigs.append(prePickConfig)

        return fingerPickPoses, fingerPickConfigs, fingerLocalPoses, fingerPrePickPoses, fingerPrePickConfigs


    def getPrePickPose(self, pickPose, handType):
        ### this function return pre-grasp poses and their correpsonding arm configs based on the pick poses
        ### Here a pre-grasp pose is a pose which is 10cm away from 
        ### the approaching direction (always local z-axis of the gripper) of its corresponding grasp pose

        prePickConfig = None

        if handType == "Left":
            ee_idx = self.robot.left_ee_idx
        else:
            ee_idx = self.robot.right_ee_idx

        temp_quat = pickPose[3:7]
        ### first change the quaternion to rotation matrix
        temp_rot_matrix = p.getMatrixFromQuaternion(temp_quat)
        ### local z-axis of the end effector
        temp_approaching_direction = [temp_rot_matrix[2], temp_rot_matrix[5], temp_rot_matrix[8]]
        temp_pos = list(np.array(pickPose[0:3]) - 0.1*np.array(temp_approaching_direction))

        prePickPose = temp_pos + temp_quat

        ### step 2: get the corresponding IK for the currently examined pose
        q_preGraspIK = p.calculateInverseKinematics(bodyUniqueId=self.robot.motomanGEO_p, 
                                endEffectorLinkIndex=ee_idx, 
                                targetPosition=prePickPose[0:3], 
                                targetOrientation=prePickPose[3:7], 
                                lowerLimits=self.robot.ll, upperLimits=self.robot.ul, jointRanges=self.robot.jr, 
                                maxNumIterations=20000, residualThreshold=0.0000001,
                                physicsClientId=self.servers[0])

        ### step 3: check the validity of the IK
        isValid = self.planner.checkIK(
            q_preGraspIK, ee_idx, prePickPose[0:3], self.robot, self.workspace)
        
        if isValid:
            if handType == "Left":
                prePickConfig = list(q_preGraspIK[0:7])
            else:
                prePickConfig = list(q_preGraspIK[7:14])

        return prePickPose, prePickConfig


    def loadObjects_e(self, clientID):

        objectsPoses = []

        for objName in self.objectsInfos:
            pos = self.objectsInfos[objName][1]
            angles = self.objectsInfos[objName][2]
            euler_angles_in_radian = [i * math.pi/180 for i in angles]
            quat = p.getQuaternionFromEuler(euler_angles_in_radian)
            obj_meshID = p.loadURDF(
                    fileName="./object_urdfs/"+objName+".urdf", 
                    basePosition=pos, baseOrientation=quat,
                    physicsClientId=clientID)
            temp_mesh = ObjectMesh(obj_meshID, objName, pos, quat, angles, self.objectsInfos[objName][0])
            objectsPoses.append(temp_mesh)

        return objectsPoses


    def loadObjects_p(self, clientID):

        massList = {
            "003_cracker_box": 2.8,
            "004_sugar_box": 1.7,
            "005_tomato_soup_can": 3.5,
            "006_mustard_bottle": 1.9,
            "008_pudding_box": 1.1,
            "009_gelatin_box": 0.8,
            "010_potted_meat_can": 2.8,
            "011_banana": 1.0,
            "019_pitcher_base": 1.6,
            "021_bleach_cleanser": 2.7
        }

        objectsPoses = []

        for objName in self.objectsInfos:
            _c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
                            fileName=self.objectsInfos[objName][0], meshScale=[1, 1, 1], physicsClientId=clientID)
            _v = p.createVisualShape(shapeType=p.GEOM_MESH, 
                            fileName=self.objectsInfos[objName][0], meshScale=[1, 1, 1], physicsClientId=clientID)
            pos = self.objectsInfos[objName][1]
            angles = self.objectsInfos[objName][2]
            euler_angles_in_radian = [i * math.pi/180 for i in angles]
            quat = p.getQuaternionFromEuler(euler_angles_in_radian)
            # _m = p.createMultiBody(baseMass=massList[objName], baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
            #                                       basePosition=pos, baseOrientation=quat, physicsClientId=clientID)
            _m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
                                                basePosition=pos, baseOrientation=quat, physicsClientId=clientID)
            temp_mesh = ObjectMesh(_m, objName, pos, quat, angles, self.objectsInfos[objName][0])
            objectsPoses.append(temp_mesh)

        return objectsPoses


    def getObjectsInfos(self, scene_index):
        ground_truth_path = os.getcwd() + "/ground_truth_poses/" + scene_index + ".txt"
        objectsInfos = OrderedDict()
        f = open(ground_truth_path)
        temp_counter = 0
        for line in f:
            line = line.split()
            if temp_counter == 0:
                targetObjectName = line[0]
            objectsInfos[line[0]] = []
            objectsInfos[line[0]].append("/mesh/"+line[0]+"/google_16k/textured.obj") ### put in the mesh file path
            objectsInfos[line[0]].append([float(line[1]), float(line[2]), float(line[3])]) ### put in the obj pos info   
            objectsInfos[line[0]].append([float(line[4]), float(line[5]), float(line[6])]) ### put in the obj orient info
            temp_counter += 1
        # for objName in objectsInfos:
        #   print( objName + ": " + str(objectsInfos[objName][1]) + ", " + str(objectsInfos[objName][2]) )

        return objectsInfos, targetObjectName


    def genServers(self, exp_mode):
        ### experiment mode: use this when you perform large-scale experiment
        ### in this case, you don't want to see any GUI since they can slow your experiment
        if exp_mode == "e":
            planningServer = p.connect(p.DIRECT)
            executingServer = p.connect(p.DIRECT)
        ### visualization mode: use this when you only want to see 
        ### the visualization of the execution and the ground truth
        elif exp_mode == "v":
            planningServer = p.connect(p.DIRECT)
            executingServer = p.connect(p.GUI)
        ### planning mode: use this when you want to see how planning goes
        elif exp_mode == "p":
            planningServer = p.connect(p.GUI)
            executingServer = p.connect(p.DIRECT)


        return [planningServer, executingServer]


class ObjectMesh:
    def __init__(self, m, objName, pos, quat, angles, meshFile):
        self.m = m
        self.objName = objName
        self.pos = pos
        self.quat = quat
        self.angles = angles
        self.meshFile = meshFile

    def update_m(self, m):
        self.m = m

    def update_pos(self, pos):
        self.pos = pos

    def update_quat(self, quat):
        self.quat = quat

    def update_angles(self, angles):
        self.angles = angles