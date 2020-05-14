from __future__ import division
import pybullet as p
import pybullet_data
import utils_poses

import math
import random
import time
import numpy as np

import sys
import os
import subprocess
import shutil

from scipy import spatial
import cPickle as pickle

import IPython
from collections import OrderedDict

if __name__ == '__main__':
	
	img_index = sys.argv[1]
	visual_mode = sys.argv[2]
	isRobot = sys.argv[3]

	if visual_mode == "po":
		planningServer = p.connect(p.GUI)
		executingServer = p.connect(p.DIRECT)
	elif visual_mode == "ex":
		planningServer = p.connect(p.DIRECT)
		executingServer = p.connect(p.GUI)


	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	### set the real-time physics simulation ###
	# p.setGravity(0.0, 0.0, -9.8, executingServer)
	# p.setRealTimeSimulation(1, executingServer)

	known_geometries_planning = []
	known_geometries_executing = []
	
	### Introduce Motoman robot ###
	if isRobot == "r":
		motomanID_p = p.loadURDF("motoman.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION, physicsClientId=planningServer)
		motomanID_e = p.loadURDF("motoman.urdf", useFixedBase=True, physicsClientId=executingServer)
		known_geometries_planning.append(motomanID_p)
		known_geometries_executing.append(motomanID_e)
	### reset the base of motoman
	motomanBasePosition = [0, 0, 0]
	motomanBaseOrientation = [0, 0, 0, 1]
	### set motoman home configuration
	home_configuration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


	############################### information related to Motoman arm###################################
	### preserve the following five lines for test purposes ###
	# print "Motoman Robot: " + str(motomanID_p)
	# num_joints = p.getNumJoints(motomanID_p, planningServer)
	# print "Num of joints: " + str(num_joints)
	# for i in range(num_joints):
	# 	print(p.getJointInfo(motomanID_p, i, planningServer))
	### end-effector index
	motoman_ee_idx = 10 ### if you use left hand
	# motoman_ee_idx = 20 ### if you use right hand
	### There is a torso joint which connects the lower and upper body (-2.957 ~ 2.957)
	### But so far we decide to make that torso joint fixed
	### For each arm, there are 10 joints and 7 of them are revolute joints
	### There are total 14 revolute joints for each arm
	### lower limits for null space
	ll = [-3.13, -1.90, -2.95, -2.36, -3.13, -1.90, -3.13, -3.13, -1.90, -2.95, -2.36, -3.13, -1.90, -3.13]
	### upper limits for null space
	ul = [3.13, 1.90, 2.95, 2.36, 3.13, 1.90, 3.13, 3.13, 1.90, -2.95, 2.36, 3.13, 1.90, 3.13]
	### joint ranges for null space
	jr = [6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26, 6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26]
	### restposes for null space
	rp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	#####################################################################################################

	################################# table scene #####################################################
	print "---------Enter to table scene!----------"
	### create the known geometries - standingBase  ###
	standingBase_dim = np.array([0.915, 0.62, 0.19])
	standingBasePosition = [motomanBasePosition[0], motomanBasePosition[1], motomanBasePosition[2]-standingBase_dim[2]/2-0.005]
	standingBase_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
								halfExtents=standingBase_dim/2, physicsClientId=planningServer)
	standingBase_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, 
								halfExtents=standingBase_dim/2, physicsClientId=planningServer)
	standingBaseM_p = p.createMultiBody(baseCollisionShapeIndex=standingBase_c_p, baseVisualShapeIndex=standingBase_v_p,
								basePosition=standingBasePosition, physicsClientId=planningServer)
	standingBase_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
								halfExtents=standingBase_dim/2, physicsClientId=executingServer)
	standingBase_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, 
								halfExtents=standingBase_dim/2, physicsClientId=executingServer)
	standingBaseM_e = p.createMultiBody(baseCollisionShapeIndex=standingBase_c_e, baseVisualShapeIndex=standingBase_v_e,
								basePosition=standingBasePosition, physicsClientId=executingServer)
	known_geometries_planning.append(standingBaseM_p)
	known_geometries_executing.append(standingBaseM_e)
	print "standing base: " + str(standingBaseM_e)
	### create the known geometries - table ###
	# table_dim = np.array([0.58, 1.44, 0.58+standingBase_dim[2]])
	table_dim = np.array([0.58, 1.32, 0.58+standingBase_dim[2]+0.005])
	# tablePosition = [0.51+table_dim[0]/2, 0.78-table_dim[1]/2, motomanBasePosition[2]+(table_dim[2]/2-standingBase_dim[2]-0.005)]
	tablePosition = [0.51+table_dim[0]/2, motomanBasePosition[1], motomanBasePosition[2]+(table_dim[2]/2-standingBase_dim[2]-0.005)]
	table_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX,
						halfExtents=table_dim/2, physicsClientId=planningServer)
	table_v_p = p.createVisualShape(shapeType=p.GEOM_BOX,
							halfExtents=table_dim/2, physicsClientId=planningServer)
	tableM_p = p.createMultiBody(baseCollisionShapeIndex=table_c_p, baseVisualShapeIndex=table_v_p,
										basePosition=tablePosition, physicsClientId=planningServer)
	table_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX,
							halfExtents=table_dim/2, physicsClientId=executingServer)
	table_v_e = p.createVisualShape(shapeType=p.GEOM_BOX,
							halfExtents=table_dim/2, physicsClientId=executingServer)
	tableM_e = p.createMultiBody(baseCollisionShapeIndex=table_c_e, baseVisualShapeIndex=table_v_e,
										basePosition=tablePosition, physicsClientId=executingServer)
	known_geometries_planning.append(tableM_p)
	known_geometries_executing.append(tableM_e)
	print "table: " + str(tableM_e)
	####################################### end of table scene setup ##########################################

	##################################### camera specification #############################################
	# print("---------camera information---------")
	camera_extrinsic = np.array(
		[[-0.0182505, -0.724286,  0.689259, 0.329174], 
		 [-0.999453,  0.0322427,  0.00741728,  -0.036492],
		 [-0.0275958, -0.688746, -0.724478, 1.24839], 
		 [0.0, 0.0, 0.0, 1.0]])


	viewMatrix = p.computeViewMatrix(
		cameraEyePosition=[camera_extrinsic[0][3], camera_extrinsic[1][3], camera_extrinsic[2][3]],
		cameraTargetPosition=[tablePosition[0]+table_dim[0]/2+0.3, tablePosition[1], tablePosition[2]+table_dim[2]/2],
		cameraUpVector=[-camera_extrinsic[0][1], -camera_extrinsic[1][1], -camera_extrinsic[2][1]])

	projectionMatrix = p.computeProjectionMatrixFOV(
		fov=90.0,
		aspect=1,
		nearVal=0.4,
		farVal=3.47)

	target_objects_dim = {
		"009_gelatin_box": [0.07, 0.09, 0.026], 
		"008_pudding_box": [0.11, 0.09, 0.035],
		"010_potted_meat_can": [0.095, 0.056, 0.085]
	}

	hypotheses, mostPromisingHypoIdxes, nObjectInPlanning = utils_poses.planScene_generation(img_index, camera_extrinsic, planningServer)

	# utils_poses.recordHypos(hypotheses, Objects, img_index)
	### wait until keybord exit
	time.sleep(10000)