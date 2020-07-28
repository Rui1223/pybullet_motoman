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
import threading
import shutil

from MotomanRobot import MotomanRobot
from Workspace import Workspace
from Camera import AzureKineticCamera
from MotionPlanner import MotionPlanner
from MotionExecutor import MotionExecutor


class cameraThread(threading.Thread):
    def __init__(self, threadID, threadName, timeInterval, robot, camera, executor, server, \
                        saveImages, collectData, scene_index):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.threadName = threadName
        self.timeInterval = timeInterval
        self.robot = robot
        self.camera = camera
        self.executor = executor
        self.server = server
        self.saveImages = saveImages
        self.collectData = collectData

    def run(self):
        print("turn on your camera to work and start to collect data")
        # self.camera.takeImage(self.name, self.timeInterval, self.server, self.saveImages)
        self.dataCollect()
        print("stop sensor function and data collection")

    def dataCollect(self):
        self.counter = 0
        while self.camera.cameraFinish == False:
            self.counter += 1
            self.right_ee_translation, self.right_ee_rotation = self.executor.get_ee_pos(self.robot, "Right")
            self.left_ee_translation, self.left_ee_rotation = self.executor.get_ee_pos(self.robot, "Left")
            self.grasp_status = [int(self.executor.isObjectInLeftHand), int(self.executor.isObjectInRightHand)]
            self.camera.takeImage(self.server, self.saveImages, self.counter)
            # print(str([self.right_ee_translation, self.right_ee_rotation]))
            # print(str([self.left_ee_translation, self.left_ee_rotation]))
            # print(self.grasp_status)
            if self.collectData:
                self.doc_yaml(self.counter)
            time.sleep(self.timeInterval)
       

    def doc_yaml(self, counter):
        yaml_file = self.camera.data_path + "/" + "frame" + str(counter) + ".yml"
        f_yaml = open(yaml_file, "w+")
        f_yaml.write("%" + "YAML:1.0\n")
        f_yaml.write("frameid: " + str(counter) + "\n")
        f_yaml.write(
            "right_ee_rotation: [" + str(self.right_ee_rotation[0]) + ", " + \
                str(self.right_ee_rotation[1]) + ", " + str(self.right_ee_rotation[2]) + ", " + \
                str(self.right_ee_rotation[3]) + "]\n")
        f_yaml.write(
            "right_ee_translation: [" + str(self.right_ee_translation[0]) + ", " + \
            str(self.right_ee_translation[1]) + ", " + str(self.right_ee_translation[2]) + "]\n")
        f_yaml.write(
            "left_ee_rotation: [" + str(self.left_ee_rotation[0]) + ", " + \
                str(self.left_ee_rotation[1]) + ", " + str(self.left_ee_rotation[2]) + ", " + \
                str(self.left_ee_rotation[3]) + "]\n")
        f_yaml.write(
            "left_ee_translation: [" + str(self.left_ee_translation[0]) + ", " + \
            str(self.left_ee_translation[1]) + ", " + str(self.left_ee_translation[2]) + "]\n")
        f_yaml.write("grasp_status: [" + str(self.grasp_status[0]) + ", " + str(self.grasp_status[1]) + "]")

        f_yaml.close()


class Experiment(object):
    ### This Experiment class defines how a experiment is performed
    def __init__(self, scene_index, exp_mode, saveImages, collectData):
        self.scene_index = scene_index
        self.exp_mode = exp_mode
        self.saveImages = saveImages
        self.collectData = collectData
        ### get two servers, planning and execution respectively
        self.servers = self.genServers(exp_mode)
        p.setGravity(0.0, 0.0, 0.0, physicsClientId=self.servers[1])
        # p.setRealTimeSimulation(enableRealTimeSimulation=0, physicsClientId=self.servers[1])
        ### set the robot ready
        self.robot = MotomanRobot(self.servers)
        ### set the workspace (table, shelf, or whatever specified)
        self.workspace = Workspace("Table", self.robot.BasePosition, self.servers)
        ### get all the geometries from the robot and the workspace
        self.planningGeometries = self.robot.known_geometries_planning + self.workspace.known_geometries_planning
        self.executingGeometries = self.robot.known_geometries_executing + self.workspace.known_geometries_executing
        ### set the camera ready
        self.camera = AzureKineticCamera(self.scene_index, self.workspace.tablePosition, self.workspace.table_dim)

        ### launch the MotionPlanner and MotionExecutor 
        self.planner = MotionPlanner(self.servers[0], self.scene_index) ### load the roadmap
        self.executor = MotionExecutor(self.servers[1], self.camera, \
            isObjectInLeftHand=False, isObjectInRightHand=False, objectInLeftHand=None, objectInRightHand=None)

        ####### specify some key poses of the arms (placement, handoff, recover) #######
        ### specify the pose for placing the object for the left arm
        self.specifyKeyEEposes()



    def runExperiment(self):
        ### get objects information from a txt file
        self.objectsInfos, self.targetObjectName = self.getObjectsInfos(self.scene_index)
        self.planObjsPoses = self.loadObjects_meshBody(self.servers[0])
        self.realObjsPoses = self.loadObjects_meshBody(self.servers[1])
        self.object_to_manipulate_p = self.planObjsPoses[0]
        self.object_to_manipulate_e = self.realObjsPoses[0]

        #### Let's filter out invalid vacuum pick poses
        self.getVacuumPickPoses(self.object_to_manipulate_p)
        self.getVacuumPrePickPaths(self.robot.leftArmCurrConfiguration_p)


        ####################################################################################################
        '''
        ### for the first local pick
        ### let's demonstrate the left arm successful picking task
        leftPick_id = 0
        ### planning the path for the left arm from picking to placing
        self.vacuumPlacementPath = []
        temp_path = self.planner.shortestPathPlanning(
                            self.vacuumPickConfigs[leftPick_id], self.vacuumPlacementConfig, \
                            "LeftPlace_"+str(leftPick_id), self.robot, self.workspace, "Left")
        self.vacuumPlacementPath.append(temp_path)
        ### planning the path for the left arm from placing to finishing
        self.vacuumFinishPath = []
        temp_path = self.planner.shortestPathPlanning(
                            self.vacuumPlacementConfig, self.vacuumFinishConfig, \
                            "LeftFinish_"+str(leftPick_id), self.robot, self.workspace, "Left")
        self.vacuumFinishPath.append(temp_path)

        raw_input("enter to show the motions for the tasks")
        ### Let's execute the path for this task
        self.executor.executePath(self.vacuumPrePickPaths[leftPick_id], self.robot, "Left")
        self.executor.executePath(
            [self.vacuumPrePickConfigs[leftPick_id], self.vacuumPickConfigs[leftPick_id]], self.robot, "Left")
        # self.executor.local_move(
        #     self.vacuumPrePickPoses[leftPick_id], self.vacuumPickPoses[leftPick_id], self.robot, "Left")

        self.executor.attachTheObject(
                        self.robot, self.object_to_manipulate_e, "Left", self.vacuumLocalPoses[leftPick_id])
        self.executor.executePath(self.vacuumPlacementPath[0], self.robot, "Left")
        time.sleep(1)
        self.executor.disattachTheObject("Left")
        p.changeDynamics(
            bodyUniqueId=self.object_to_manipulate_e.m, linkIndex=-1, mass=1.0, physicsClientId=self.servers[1])
        p.setGravity(0.0, 0.0, -9.8, physicsClientId=self.servers[1])
        p.setRealTimeSimulation(enableRealTimeSimulation=1, physicsClientId=self.servers[1])
        time.sleep(2)
        p.setRealTimeSimulation(enableRealTimeSimulation=0, physicsClientId=self.servers[1])        
        self.executor.executePath(self.vacuumFinishPath[0], self.robot, "Left")
        '''
        #######################################################################################################
        

        #######################################################################################################
        
        ### for the second local pick
        ### let's demonstrate the left arm handoff to righ arm and right arm place the object
        leftPick_id = 1
        ### planning the path for the left arm from picking to handoff
        self.vacuumHandoffPath = []
        temp_path = self.planner.shortestPathPlanning(
                            self.vacuumPickConfigs[leftPick_id], self.vacuumHandoffConfig, \
                            "LeftHandOff_"+str(leftPick_id), self.robot, self.workspace, "Left")
        self.vacuumHandoffPath.append(temp_path)
        ### planning the path for the left arm from placing to finishing
        self.vacuumFinishPath = []
        temp_path = self.planner.shortestPathPlanning(
                            self.vacuumHandoffConfig, self.vacuumFinishConfig, \
                            "LeftFinish_"+str(leftPick_id), self.robot, self.workspace, "Left")
        self.vacuumFinishPath.append(temp_path)

        ### set both the arm and the object to that config
        self.robot.setSingleArmToConfig(self.vacuumHandoffConfig, "Left")
        self.robot.updateLeftArmConfig(self.vacuumHandoffConfig, self.servers[0])
        self.updatePlanningObjectBasedonLocalPose(
                        self.object_to_manipulate_p, self.vacuumLocalPoses[leftPick_id], "Left")

        #### Let's filter out invalid fingered pick poses
        self.getFingerPickPoses(self.object_to_manipulate_p)
        self.getFingerPrePickPaths(self.robot.rightArmCurrConfiguration_p)

        ### planning the path for the right arm from picking to placement
        self.fingerPlacementPath = []
        temp_path = self.planner.shortestPathPlanning(
                            self.fingerPrePickConfigs[0], self.fingerPlacementConfig, \
                            "RightPlacem_"+str(leftPick_id), self.robot, self.workspace, "Right")
        self.fingerPlacementPath.append(temp_path)
        
        
        raw_input("enter to show the motions for the tasks")

        ## Set up the camera
        self.dataCollector = cameraThread(
                1, "camera thread", 0.1, self.robot, self.camera, self.executor, self.servers[1], \
                self.saveImages, self.collectData, self.scene_index)
        self.dataCollector.start()

        ### Let's execute the path for this task
        self.executor.executePath(self.vacuumPrePickPaths[leftPick_id], self.robot, "Left")
        self.executor.executePath(
            [self.vacuumPrePickConfigs[leftPick_id], self.vacuumPickConfigs[leftPick_id]], self.robot, "Left")
        # self.executor.local_move(
        #     self.vacuumPrePickPoses[leftPick_id], self.vacuumPickPoses[leftPick_id], self.robot, "Left")
        self.executor.attachTheObject(
                        self.robot, self.object_to_manipulate_e, "Left", self.vacuumLocalPoses[leftPick_id])
        self.executor.executePath(self.vacuumHandoffPath[0], self.robot, "Left")
        time.sleep(1)

        self.executor.executePath(self.fingerPrePickPaths[0], self.robot, "Right")
        self.executor.local_move(
            self.fingerPrePickPoses[0], self.fingerPickPoses[0], self.robot, "Right")
        time.sleep(1)
        self.executor.attachTheObject(self.robot, self.object_to_manipulate_e, "Right", self.fingerLocalPoses[0])
        time.sleep(1)
        self.executor.disattachTheObject("Left")

        self.executor.executePath(self.vacuumFinishPath[0], self.robot, "Left")
        time.sleep(1)
        self.executor.executePath(self.fingerPlacementPath[0], self.robot, "Right")   
        time.sleep(1)
        self.executor.disattachTheObject("Right")
        ### add some mass to the object so as to let it fall
        p.changeDynamics(
            bodyUniqueId=self.object_to_manipulate_e.m, linkIndex=-1, mass=1.0, physicsClientId=self.servers[1])
        p.setGravity(0.0, 0.0, -9.8, physicsClientId=self.servers[1])
        p.setRealTimeSimulation(enableRealTimeSimulation=1, physicsClientId=self.servers[1])
        time.sleep(2)
        p.setRealTimeSimulation(enableRealTimeSimulation=0, physicsClientId=self.servers[1])

        self.camera.cameraFinish = True
        
        #######################################################################################################


    def specifyKeyEEposes(self):
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


    def getObjectGlobalPose(self, local_pose, ee_global_pose):
        local_pose1 = p.invertTransform(local_pose[0:3], local_pose[3:7])
        
        temp_object_global_pose = p.multiplyTransforms(
            ee_global_pose[0:3], ee_global_pose[3:7],
            list(local_pose1[0]), list(local_pose1[1])
        )

        object_global_pose = list(temp_object_global_pose[0]) + list(temp_object_global_pose[1])

        return object_global_pose


    def getVacuumPickPoses(self, targetObj):
        ### the vacuum gripper is mounted on the left arm
        self.vacuum_pickPose_file = os.path.join(os.getcwd(), "gripper_poses", self.scene_index, "vacuumPickPoses.txt")
        self.vacuumPickPoses = []
        self.vacuumPickConfigs = []
        self.vacuumLocalPoses = []
        self.vacuumPrePickPoses = []
        self.vacuumPrePickConfigs = []

        f = open(self.vacuum_pickPose_file, "r")
        for line in f:
            line = line.split(',')
            temp_pose = [float(temp_e) for temp_e in line]
            ########## check this pose ##########
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

            if isValid:
                ### generate pre-grasp pose and check its validity
                prePickPose, prePickConfig = self.getPrePickPose(global_pose, "Left")
                if prePickConfig != None:
                    ### the pre-grasp is also valid
                    ### Now let's add the poses and configs
                    self.vacuumPickPoses.append(global_pose)
                    self.vacuumPickConfigs.append(list(q_graspIK[0:7]))
                    self.vacuumLocalPoses.append(
                        temp_pose[0:3] + list(p.getQuaternionFromEuler([i*math.pi/180 for i in temp_pose[3:6]])))
                    self.vacuumPrePickPoses.append(prePickPose)
                    self.vacuumPrePickConfigs.append(prePickConfig)


    def getVacuumPrePickPaths(self, currArmConfig):
        self.vacuumPrePickPaths = []
        for i in range(len(self.vacuumPrePickConfigs)):
            temp_path = self.planner.shortestPathPlanning(
                            currArmConfig, self.vacuumPrePickConfigs[i], "LeftPrePick_"+str(i), 
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


    def getFingerPickPoses(self, targetObj):
        ### the fingered gripper is mounted on the right arm
        self.finger_pickPose_file = os.path.join(os.getcwd(), "gripper_poses", self.scene_index, "fingerPickPoses.txt")
        self.fingerPickPoses = []
        self.fingerPickConfigs = []
        self.fingerLocalPoses = []
        self.fingerPrePickPoses = []
        self.fingerPrePickConfigs = []

        f = open(self.finger_pickPose_file, "r")
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
                ### generate pre-grasp pose and check its validity
                prePickPose, prePickConfig = self.getPrePickPose(global_pose, "Right")
                # raw_input("let's see next grasp")
                if prePickConfig != None:
                    ### the pre-grasp is also valid
                    ### Now let's add the poses and configs                    
                    self.fingerPickPoses.append(global_pose)
                    self.fingerPickConfigs.append(list(q_graspIK[0:7]))
                    self.fingerLocalPoses.append(
                        temp_pose[0:3] + list(p.getQuaternionFromEuler([i*math.pi/180 for i in temp_pose[3:6]])))
                    self.fingerPrePickPoses.append(prePickPose)
                    self.fingerPrePickConfigs.append(prePickConfig)


    def getFingerPrePickPaths(self, currArmConfig):
        self.fingerPrePickPaths = []
        for i in range(len(self.fingerPrePickConfigs)):
            temp_path = self.planner.shortestPathPlanning(
                            currArmConfig, self.fingerPrePickConfigs[i], "rightPrePick_"+str(i), 
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



    def updatePlanningObjectBasedOnRealScene(self):

        ls = p.getBasePositionAndOrientation(self.object_to_manipulate_e.m, physicsClientId=self.servers[1])
        print("ls: " + str(ls))
        object_global_pose = list(ls[0]) + list(ls[1])
        ### remove the old mesh of the grasped objectg
        self.removeOldMesh(self.object_to_manipulate_p, self.servers[0])
        self.addNewMesh(self.object_to_manipulate_p, object_global_pose, self.servers[0])        



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



    def loadObjects_urdf(self, clientID):

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


    def loadObjects_meshBody(self, clientID):

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
        ground_truth_object_file = os.getcwd() + "/ground_truth_objectsPlacement/" + scene_index + ".txt"
        objectsInfos = OrderedDict()
        f = open(ground_truth_object_file)
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