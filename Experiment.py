from __future__ import division
import pybullet as p
import pybullet_data

from collections import OrderedDict
import math
import numpy as np
import os

from MotomanRobot import MotomanRobot
from Workspace import Workspace
from Camera import AzureKineticCamera
from MotionPlanner import MotionPlanner

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
        ### set the camera ready
        self.camera = AzureKineticCamera(self.scene_index, self.workspace.tablePosition, self.workspace.table_dim)

        ### get objects information from a txt file
        self.ObjectsInfos, self.targetObjectName = self.getObjectsInfos(scene_index)
        self.objectsPoses = self.loadObjects(self.server[0])

        ### Take the initial image
        self.camera.takeImage(self.server[0], self.saveImages)

        ### Let's generate a roadmap for any future planner to work on
        self.planner = MotionPlanner(1500, self.server[0], scene_index)
        self.planner.PRMstar_generation(self.workspace, self.robot, "Right")












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

        for objName in self.ObjectsInfos:
            _c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
                                        fileName=self.ObjectsInfos[objName][0], meshScale=[1, 1, 1], physicsClientId=clientID)
            _v = p.createVisualShape(shapeType=p.GEOM_MESH, 
                                        fileName=self.ObjectsInfos[objName][0], meshScale=[1, 1, 1], physicsClientId=clientID)
            pos = self.ObjectsInfos[objName][1]
            angles = self.ObjectsInfos[objName][2]
            euler_angles_in_radian = [i * math.pi/180 for i in angles]
            quat = p.getQuaternionFromEuler(euler_angles_in_radian)
            # _m = p.createMultiBody(baseMass=massList[objName], baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
            #                                       basePosition=pos, baseOrientation=quat, physicsClientId=clientID)
            _m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
                                                basePosition=pos, baseOrientation=quat, physicsClientId=clientID)
            temp_mesh = ObjectMesh(_m, objName, pos, quat, angles)
            objectsPoses.append(temp_mesh)

        return objectsPoses


    def getObjectsInfos(self, scene_index):
        ground_truth_path = os.getcwd() + "/ground_truth_poses/" + scene_index + ".txt"
        ObjectsInfos = OrderedDict()
        f = open(ground_truth_path)
        temp_counter = 0
        for line in f:
            line = line.split()
            if temp_counter == 0:
                targetObjectName = line[0]
            ObjectsInfos[line[0]] = []
            ObjectsInfos[line[0]].append("/mesh/"+line[0]+"/google_16k/textured.obj") ### put in the mesh file path
            ObjectsInfos[line[0]].append([float(line[1]), float(line[2]), float(line[3])]) ### put in the obj pos info   
            ObjectsInfos[line[0]].append([float(line[4]), float(line[5]), float(line[6])]) ### put in the obj orient info
            temp_counter += 1
        # for objName in ObjectsInfos:
        #   print( objName + ": " + str(ObjectsInfos[objName][1]) + ", " + str(ObjectsInfos[objName][2]) )

        return ObjectsInfos, targetObjectName


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
    def __init__(self, m, objName, pos, quat, angles):
        self.m = m
        self.objName = objName
        self.pos = pos
        self.quat = quat
        self.angles = angles