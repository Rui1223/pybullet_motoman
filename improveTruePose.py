from __future__ import division 
import pybullet as p
import numpy as np
import pybullet_data
import IPython
import math
import time
import random

import os
import sys

from scipy import spatial
import cPickle as pickle
from collections import OrderedDict

class TrueMesh:
	def __init__(self, m, objName, pos, quat):
		self.m = m
		self.objName = objName
		self.pos = pos
		self.quat = quat

	def setHeight(self, h):
		self.pos[2] = h


def getHeight(id):
	return list(p.getBasePositionAndOrientation(id)[0])[2]


def read_print_poses(img_index):
	pose_info_path = "/home/rui/Documents/research/instance_segmentation/my_dataset/poses_quaternion/" + img_index + ".txt"
	Objects = OrderedDict()
	f = open(pose_info_path)
	temp_counter = 0
	for line in f:
		line = line.split()
		if temp_counter == 0:
			targetObjectName = line[0]
		Objects[line[0]] = []
		Objects[line[0]].append("/mesh/"+line[0]+"/google_16k/textured.obj") ### put in the mesh file path
		Objects[line[0]].append([float(line[1]), float(line[2]), float(line[3])]) ### put in the obj pos info
		quat = [float(line[5]), float(line[6]), float(line[7]), float(line[4])]
		Objects[line[0]].append(quat) ### put in the obj orient info (quat)
		angles = p.getEulerFromQuaternion(quat)
		angles = [i*180/math.pi for i in angles]
		Objects[line[0]].append(angles)
		temp_counter += 1
	for objName in Objects:
		print( objName + ": " + str(Objects[objName][1]) + ", " + str(Objects[objName][3]) )

	f.close()

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
		quat = Objects[objName][2]
		_m = p.createMultiBody(baseMass=massList[objName], baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
												basePosition=pos, baseOrientation=quat, physicsClientId=clientId)
		# _m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
		# 									basePosition=pos, baseOrientation=quat, physicsClientId=clientId)
		temp_mesh = TrueMesh(_m, objName, pos, quat)
		truePoses.append(temp_mesh)

	p.setGravity(0.0, 0.0, -9.8, clientId)
	p.setRealTimeSimulation(1, clientId)

		
	print "Number of Objects in the ground truth (execution): " + str(len(truePoses))
	return truePoses, nObjectInExecuting


def updateAndSaveTruePoses(truePoses, clientId):
	new_file_path = "/home/rui/Documents/research/instance_segmentation/my_dataset/poses_info/" + img_index + ".txt"
	f_new = open(new_file_path, "w")

	print("\nupdated poses after turn on physics\n")

	for truePose in truePoses:
		pos = list(p.getBasePositionAndOrientation(truePose.m, clientId)[0])
		quat = list(p.getBasePositionAndOrientation(truePose.m, clientId)[1])
		angles = p.getEulerFromQuaternion(quat)
		angles = [i*180/math.pi for i in angles]
		print( truePose.objName + ": " + str(pos) + ", " + str(angles) )
		f_new.write(truePose.objName + " " + format(pos[0], '.3f') + " " + format(pos[1], '.3f') + " " + format(pos[2], '.3f') + " ")
		f_new.write(format(angles[0], '.3f') + " " + format(angles[1], '.3f') + " " + format(angles[2], '.3f') + "\n")

	f_new.close()



if __name__ == '__main__':
	img_index = sys.argv[1]
	origin_file_path = "/home/rui/Documents/research/instance_segmentation/my_dataset/poses_quaternion/" + img_index + ".txt"


	executingServer = p.connect(p.GUI)
	p.setAdditionalSearchPath(pybullet_data.getDataPath())


	known_geometries_executing = []

	################################# table scene #####################################################
	### create the known geometries - table ###
	motomanBasePosition = [0, 0, 0]
	motomanBaseOrientation = [0, 0, 0, 1]
	standingBase_dim = np.array([0.915, 0.62, 0.19])
	table_dim = np.array([0.58, 1.32, 0.58+standingBase_dim[2]+0.005])
	tablePosition = [0.51+table_dim[0]/2, motomanBasePosition[1], motomanBasePosition[2]+(table_dim[2]/2-standingBase_dim[2]-0.005)]
	table_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX,
							halfExtents=table_dim/2, physicsClientId=executingServer)
	table_v_e = p.createVisualShape(shapeType=p.GEOM_BOX,
							halfExtents=table_dim/2, physicsClientId=executingServer)
	tableM_e = p.createMultiBody(baseCollisionShapeIndex=table_c_e, baseVisualShapeIndex=table_v_e,
										basePosition=tablePosition, physicsClientId=executingServer)
	known_geometries_executing.append(tableM_e)
	print "table: " + str(tableM_e)
	####################################### end of table scene setup ##########################################
	Objects, targetObjectName = read_print_poses(img_index)
	print("targetObjectName", targetObjectName)
	truePoses, nObjectInExecuting = trueScene_generation(Objects, executingServer)
	time.sleep(2)
	updateAndSaveTruePoses(truePoses, executingServer)


	'''
	### for each true poses, let's fix the height problem
	for truePose in truePoses:
		id_pose = truePose.m
		curr_h = truePose.pos[2]
		update_h = curr_h + (p.getAABB(tableM_e)[1][2] - p.getAABB(id_pose)[0][2])
		truePose.setHeight(update_h)
	'''

	time.sleep(10000)

