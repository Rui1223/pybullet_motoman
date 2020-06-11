from __future__ import division
import pybullet as p
import pybullet_data

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
        self.server = self.genServer(exp_mode)
        ### set the robot ready
        self.robot = MotomanRobot(self.server)
        ### set the workspace (table, shelf, or whatever specified)
        self.workspace = Workspace("Table", self.robot.BasePosition, self.server)
        ### get all the geometries from the robot and the workspace
        self.planningGeometries = self.robot.known_geometries_planning + self.workspace.known_geometries_planning
        self.executingGeometries = self.robot.known_geometries_executing + self.workspace.known_geometries_executing
        ### set the camera ready
        self.camera = AzureKineticCamera(self.scene_index, self.workspace.tablePosition, self.workspace.table_dim)


    def runExperiment(self):
        ### get objects information from a txt file
        self.objectsInfos, self.targetObjectName = self.getObjectsInfos(self.scene_index)
        self.objectsPoses = self.loadObjects(self.server[0])
        self.realObjsPoses = self.loadObjects(self.server[1])
        self.objectInHand_p = self.objectsPoses[0]
        self.objectInHand_e = self.realObjsPoses[0]
        print("objectInHand_p: " + str(self.objectInHand_p.m))
        print("objectInHand_e: " + str(self.objectInHand_e.m))

        
        ### Take the initial image
        self.camera.takeImage(self.server[0], self.saveImages)

        ### Let's generate a roadmap for any future planner to work on
        startTime = time.clock()
        self.planner = MotionPlanner(self.server[0], self.scene_index)
        self.executor = MotionExecutor(self.server[1])
        print("Time spent for loading the nodes: " + str(time.clock() - startTime) + "\n")

        ### Now Let's check valid picking poses for vacuum gripper
        self.vacuum_pose_file = os.path.join(os.getcwd(), "poses", self.scene_index, "vacuumPickPoses.txt")
        self.vacuumPickPoses, self.vacuumPickConfigs, self.vacuumLocalPoses = \
                                                    self.getVacuumPickPoses(self.objectsPoses[0])
        self.vacuumPickPaths = []
        for i in range(len(self.vacuumPickConfigs)):
            # print("current vacuumPickConfigs: " + str(self.vacuumPickConfigs[i]))
            temp_path = self.planner.shortestPathPlanning(
                    self.robot.leftArmCurrConfiguration, self.vacuumPickConfigs[i], "LeftPick_"+str(i), 
                    self.robot, self.workspace, "Left")
            self.vacuumPickPaths.append(temp_path)
        for i in range(len(self.vacuumPickPaths)):
            if len(self.vacuumPickPaths[i]) == 0:
                self.vacuumPickPaths.remove(self.vacuumPickPaths[i])
                self.vacuumPickPoses.remove(self.vacuumPickPoses[i])
                self.vacuumPickConfigs.remove(self.vacuumPickConfigs[i])
                self.vacuumLocalPoses.remove(self.vacuumLocalPoses[i])


        ### specify the placement pose of the end effector
        ##################################################################################################
        self.vacuumPlacementPose = \
            [self.workspace.tablePosition[0], self.workspace.tablePosition[1], \
            self.workspace.tablePosition[2] + 0.9] + list(p.getQuaternionFromEuler([0.0, math.pi, 0.0]))
        # print("vacuumPlacementPose: " + str(self.vacuumPlacementPose))
        self.vacuumPlacementConfig = self.getVacuumPlacementPoses(self.vacuumPlacementPose)
        # self.robot.moveSingleArm(self.vacuumPlacementConfig, self.robot.motomanGEO_p, "Left", self.server[0])
        ######################################################################################################

        
        ### for the first local pick
        ### let's demonstrate the left arm successful picking task
        self.vacuumPlacementPath = []
        leftPick_id = 0
        ### planning the path for the left arm from picking to placing
        temp_start_config = self.vacuumPickConfigs[leftPick_id]
        # print("vacuumPlacementConfig: " + str(self.vacuumPlacementConfig))
        temp_path = self.planner.shortestPathPlanning(
                self.vacuumPickConfigs[leftPick_id], self.vacuumPlacementConfig, "LeftPlace_"+str(leftPick_id),
                self.robot, self.workspace, "Left")
        self.vacuumPlacementPath.append(temp_path)
        print("vacuumPlacementPath: " + str(self.vacuumPlacementPath))

        ### Let's execute the path for this task

        self.objectInHand = None
        self.executor.executePath(
            self.vacuumPickPaths[leftPick_id], self.robot, "Left", self.robot.motomanGEO_e)
        ls = p.getLinkState(
            bodyUniqueId=self.robot.motomanGEO_e, linkIndex=self.robot.left_ee_idx, physicsClientId=self.server[1])
        print("ls: " + str(ls))
        ee_global_pos = list(ls[0])
        self.executor.attachTheObject(self.robot, self.objectInHand_e, "Left", ee_global_pos)
        raw_input("object attached! Press ENTER to continue")
        self.executor.executePath(
            self.vacuumPlacementPath[0], self.robot, "Left", self.robot.motomanGEO_e)
        

        # ### check how the object looks like in this placement pose through all these valid picks
        # # for i in range(len(self.vacuumLocalPoses)):
        # for i in range(0,1):
        #     ### first determine the object location
        #     vacuumLocalPose = self.vacuumLocalPoses[i]
        #     temp_objectPose = self.getObjectPose(vacuumLocalPose, self.vacuumPlacementPose)
        #     self.updateGrapsedObject(temp_objectPose, self.server[0])
        

        ### execute one path

        '''
        raw_input("wait for ENTER")
        self.executor.executePath(self.vacuumPickPaths[0], self.robot, "Left", self.robot.motomanGEO_e)

        ### Now let's set the pose and the arm config for vacuum gripper to lift or transfer the object
        self.vacuumTransferPose = \
            [self.workspace.tablePosition[0], self.workspace.tablePosition[1], \
            self.workspace.tablePosition[2] + 1.1] + self.vacuumPickPoses[0][3:7]
        # raw_input("quick")
        self.vacuumTransferConfig = self.getVacuumTransferPoses(self.vacuumTransferPose)
        # self.robot.moveSingleArm(self.vacuumTransferConfig, self.robot.motomanGEO_p, "Left", self.server[0])
        # transferPath = self.planner.shortestPathPlanning(
        #         self.vacuumPickConfigs[0], self.vacuumTransferConfig[0:7], "LeftTransfer_"+str(0), 
        #         self.robot, self.workspace, "Left")
        transferPath = [self.vacuumPickConfigs[0], self.vacuumTransferConfig[0:7]]
        self.executor.executePath(transferPath, self.robot, "Left", self.robot.motomanGEO_e)

        ### Now let's set the pose and the arm config for vacuum gripper to drop the object
        self.vacuumDropPose = \
            [self.workspace.tablePosition[0], self.workspace.tablePosition[1], \
            self.workspace.tablePosition[2] + 0.7] + self.vacuumTransferPose[3:7]
        # raw_input("quick")
        self.vacuumDropConfig = self.getVacuumDropPoses(self.vacuumDropPose)
        # self.robot.moveSingleArm(self.vacuumTransferConfig, self.robot.motomanGEO_p, "Left", self.server[0])
        # dropPath = self.planner.shortestPathPlanning(
        #         self.vacuumTransferConfig[0:7], self.vacuumDropConfig[0:7], "LeftDrop_"+str(0), 
        #         self.robot, self.workspace, "Left")
        dropPath = [self.vacuumTransferConfig[0:7], self.vacuumDropConfig[0:7]]
        self.executor.executePath(dropPath, self.robot, "Left", self.robot.motomanGEO_e)
        '''

        '''
        ### Also check valid picking poses for finger gripper
        self.finger_pose_file = os.path.join(os.getcwd(), "poses", self.scene_index, "fingerPickPoses.txt")
        self.fingerPickPoses, self.fingerPickConfigs = self.getFingerPickPoses(self.objectsPoses[0])
        '''

    def updateGrapsedObject(self, object_global_pose, clientID):
        ### first delete current mesh of the grasped object
        if clientID == self.server[0]:
            self.removeOldMesh(self.objectsPoses[0], clientID)
            self.addNewMesh(self.objectsPoses[0], object_global_pose, clientID)
        else:
            self.removeOldMesh(self.realObjsPoses[0], clientID)
            self.addNewMesh(self.realObjsPoses[0], object_global_pose, clientID)


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
        

    def getObjectPose(self, local_pose, ee_global_pose):
        local_pose1 = p.invertTransform(local_pose[0:3], local_pose[3:7])
        
        temp_object_global_pose = p.multiplyTransforms(
            ee_global_pose[0:3], ee_global_pose[3:7],
            list(local_pose1[0]), list(local_pose1[1])
        )

        object_global_pose = list(temp_object_global_pose[0]) + list(temp_object_global_pose[1])

        return object_global_pose


    def getVacuumDropPoses(self, vacuumDropPose):
        ### get the corresponding IK for the currently examined pose
        q_dropIK = p.calculateInverseKinematics(bodyUniqueId=self.robot.motomanGEO_p, 
                                endEffectorLinkIndex=self.robot.left_ee_idx, 
                                targetPosition=vacuumDropPose[0:3], 
                                targetOrientation=vacuumDropPose[3:7], 
                                lowerLimits=self.robot.ll, upperLimits=self.robot.ul, jointRanges=self.robot.jr, 
                                maxNumIterations=20000, residualThreshold=0.0000001,
                                physicsClientId=self.server[0])

        return q_dropIK

    def getVacuumPlacementPoses(self, vacuumPlacementPose):
        ### get the corresponding IK for the currently examined pose
        q_placementIK = p.calculateInverseKinematics(bodyUniqueId=self.robot.motomanGEO_p, 
                                endEffectorLinkIndex=self.robot.left_ee_idx, 
                                targetPosition=vacuumPlacementPose[0:3], 
                                targetOrientation=vacuumPlacementPose[3:7], 
                                lowerLimits=self.robot.ll, upperLimits=self.robot.ul, jointRanges=self.robot.jr, 
                                maxNumIterations=20000, residualThreshold=0.0000001,
                                physicsClientId=self.server[0])
        return list(q_placementIK[0:7])


    def getVacuumTransferPoses(self, vacuumTransferPose):
        ### get the corresponding IK for the currently examined pose
        q_transferIK = p.calculateInverseKinematics(bodyUniqueId=self.robot.motomanGEO_p, 
                                endEffectorLinkIndex=self.robot.left_ee_idx, 
                                targetPosition=vacuumTransferPose[0:3], 
                                targetOrientation=vacuumTransferPose[3:7], 
                                lowerLimits=self.robot.ll, upperLimits=self.robot.ul, jointRanges=self.robot.jr, 
                                maxNumIterations=20000, residualThreshold=0.0000001,
                                physicsClientId=self.server[0])

        return q_transferIK


    def getVacuumPickPoses(self, targetObj):
        vacuumPickPoses = []
        vacuumPickConfigs = []
        vacuumLocalPoses = []
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
            ### step 2: get the corresponding IK for the currently examined pose
            q_graspIK = p.calculateInverseKinematics(bodyUniqueId=self.robot.motomanGEO_p, 
                                    endEffectorLinkIndex=self.robot.left_ee_idx, 
                                    targetPosition=list(global_pose[0]), 
                                    targetOrientation=list(global_pose[1]), 
                                    lowerLimits=self.robot.ll, upperLimits=self.robot.ul, jointRanges=self.robot.jr, 
                                    maxNumIterations=20000, residualThreshold=0.0000001,
                                    physicsClientId=self.server[0])

            ### step 3: check the validity of the IK
            isValid = self.planner.checkIK(
                q_graspIK, self.robot.left_ee_idx, list(global_pose[0]), self.robot, self.workspace, "Left")

            if isValid:
                vacuumPickPoses.append(list(global_pose[0]) + list(global_pose[1]))
                vacuumPickConfigs.append(list(q_graspIK[0:7]))
                vacuumLocalPoses.append(temp_pose[0:3] + list(p.getQuaternionFromEuler([i*math.pi/180 for i in temp_pose[3:6]])))


        return vacuumPickPoses, vacuumPickConfigs, vacuumLocalPoses


    def getFingerPickPoses(self, targetObj):
        fingerPickPoses = []
        fingerPickConfigs = []
        f = open(self.finger_pose_file, "r")
        for line in f:
            line = line.split('.')
            temp_pose = [float(temp_e) for temp_e in line]
            ### check this pose ###
            ### step 1: convert it from target object local frame to global frame in the workspace
            global_pose = p.multiplyTransforms(
                targetObj.pos, targetObj.quat,
                temp_pose[0:3], p.getQuaternionFromEuler([i*math.pi/180 for i in temp_pose[3:6]])
            )
            ### step 2: get the corresponding IK for the currently examined pose
            q_graspIK = p.calculateInverseKinematics(bodyUniqueId=self.robot.motomanGEO_p, 
                                    endEffectorLinkIndex=self.robot.right_ee_idx, 
                                    targetPosition=list(global_pose[0]), 
                                    targetOrientation=list(global_pose[1]), 
                                    lowerLimits=self.robot.ll, upperLimits=self.robot.ul, jointRanges=self.robot.jr, 
                                    maxNumIterations=20000, residualThreshold=0.0000001,
                                    physicsClientId=self.server[0])

            ### step 3: check the validity of the IK
            isValid = self.planner.checkIK(
                q_graspIK, self.robot.right_ee_idx, list(global_pose[0]), self.robot, self.workspace, "right")

            if isValid:
                fingerPickPoses.append(list(global_pose[0]) + list(global_pose[1]))
                fingerPickConfigs.append(list(q_graspIK[0:7]))

        return fingerPickPoses, fingerPickConfigs


    def loadObjects(self, clientID):
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

        # objectsPoses = []

        # for objName in self.objectsInfos:
        #     _c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
        #                     fileName=self.objectsInfos[objName][0], meshScale=[1, 1, 1], physicsClientId=clientID)
        #     _v = p.createVisualShape(shapeType=p.GEOM_MESH, 
        #                     fileName=self.objectsInfos[objName][0], meshScale=[1, 1, 1], physicsClientId=clientID)
        #     pos = self.objectsInfos[objName][1]
        #     angles = self.objectsInfos[objName][2]
        #     euler_angles_in_radian = [i * math.pi/180 for i in angles]
        #     quat = p.getQuaternionFromEuler(euler_angles_in_radian)
        #     # _m = p.createMultiBody(baseMass=massList[objName], baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
        #     #                                       basePosition=pos, baseOrientation=quat, physicsClientId=clientID)
        #     _m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
        #                                         basePosition=pos, baseOrientation=quat, physicsClientId=clientID)
        #     temp_mesh = ObjectMesh(_m, objName, pos, quat, angles, self.objectsInfos[objName][0])
        #     objectsPoses.append(temp_mesh)

        # return objectsPoses


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


    def genServer(self, exp_mode):
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

    # def update_m(self, m):
    #     self.m = m

    # def update_pos(self, pos):
    #     self.pos = pos

    # def update_quat(self, quat):
    #     self.quat = quat

    # def update_angles(self, angles):
    #     self.angles = angles