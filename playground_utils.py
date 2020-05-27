from __future__ import division 
import pybullet as p
import numpy as np
import pybullet_data
import os
from collections import OrderedDict
import math
from PIL import Image
from skimage.io import imread, imsave

import IPython

class TrueMesh:
    def __init__(self, m, objName, pos, quat, angles):
        self.m = m
        self.objName = objName
        self.pos = pos
        self.quat = quat
        self.angles = angles

def sensorImageTaker(viewMatrix, projectionMatrix, clientId, img_path):
    ### get the image
    width, height, rgbImg, depthImg, segImg = p.getCameraImage(
    width=1280,
    height=720,
    viewMatrix=viewMatrix,
    projectionMatrix=projectionMatrix,
    physicsClientId=clientId)

    ### save the images
    ### rgb
    imsave(img_path + "/rgb.png", rgbImg)
    ### depth
    imsave(img_path + "/depth.png", depthImg)
    ### segmentation
    imsave(img_path + "/segment.png", segImg*1000)

def read_print_poses(scene_index):
    ground_truth_path = os.getcwd() + "/ground_truth_poses/" + scene_index + ".txt"
    Objects = OrderedDict()
    f = open(ground_truth_path)
    temp_counter = 0
    for line in f:
        line = line.split()
        if temp_counter == 0:
            targetObjectName = line[0]
        Objects[line[0]] = []
        Objects[line[0]].append("/mesh/"+line[0]+"/google_16k/textured.obj") ### put in the mesh file path
        Objects[line[0]].append([float(line[1]), float(line[2]), float(line[3])]) ### put in the obj pos info   
        Objects[line[0]].append([float(line[4]), float(line[5]), float(line[6])]) ### put in the obj orient info
        temp_counter += 1
    # for objName in Objects:
    #   print( objName + ": " + str(Objects[objName][1]) + ", " + str(Objects[objName][2]) )

    return Objects, targetObjectName

def trueScene_generation(Objects, clientId):
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

    truePoses = []
    nObjectInExecuting = len(Objects)

    for objName in Objects:
        _c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
                                    fileName=Objects[objName][0], meshScale=[1, 1, 1], physicsClientId=clientId)
        _v = p.createVisualShape(shapeType=p.GEOM_MESH, 
                                    fileName=Objects[objName][0], meshScale=[1, 1, 1], physicsClientId=clientId)
        pos = Objects[objName][1]
        angles = Objects[objName][2]
        euler_angles_in_radian = [i * math.pi/180 for i in angles]
        quat = p.getQuaternionFromEuler(euler_angles_in_radian)
        # _m = p.createMultiBody(baseMass=massList[objName], baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
        #                                       basePosition=pos, baseOrientation=quat, physicsClientId=clientId)
        _m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
                                            basePosition=pos, baseOrientation=quat, physicsClientId=clientId)
        temp_mesh = TrueMesh(_m, objName, pos, quat, angles)
        truePoses.append(temp_mesh)

    # p.setGravity(0.0, 0.0, -9.8, clientId)
    # p.setRealTimeSimulation(1, clientId)
    print "Number of Objects in the ground truth (execution): " + str(len(truePoses))
    return truePoses, nObjectInExecuting



def genVacuumGrasps(targetObj, vacuum_gripper_localPicks):
    vacuum_grasps = []
    for localPick in vacuum_gripper_localPicks:
        vacuum_pick = []
        ### Let's get the new configuration of the pick in the global frame
        new_config = p.multiplyTransforms(
            targetObj.pos, targetObj.quat, 
            localPick[0:3], p.getQuaternionFromEuler([i*math.pi/180 for i in localPick[3:6]])
        )
        for item in new_config:
            vacuum_pick += list(item)
        vacuum_grasps.append(vacuum_pick)

    # print("vacuum_grasps: ")
    # for vacuum_grasp in vacuum_grasps:
    #     print(vacuum_grasp)

    return vacuum_grasps


