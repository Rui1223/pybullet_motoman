from __future__ import division 
import pybullet as p
import numpy as np
import pybullet_data
import IPython
import math
import time
import random

import os

from scipy import spatial
import cPickle as pickle
from collections import OrderedDict


class TrueMesh:
	def __init__(self, m, objName, pos, quat, angles):
		self.m = m
		self.objName = objName
		self.pos = pos
		self.quat = quat
		self.angles = angles

class Mesh:
	def __init__(self, m, objName, objIdx, hypoIdx, pos, quat, angles, prob):
		self.m = m
		self.objName = objName
		self.objIdx = objIdx
		self.hypoIdx = hypoIdx
		self.pos = pos
		self.quat = quat
		self.angles = angles
		self.prob = prob


	def setProb(self, prob):
		self.prob = prob

def rotationMatrixToQuaternion(T):
	### first compute the trace
	tr = T[0][0] + T[1][1] + T[2][2]

	if (tr > 0):
		S = math.sqrt(tr+1.0) * 2
		qw = 0.25 * S
		qx = (T[2][1] - T[1][2]) / S
		qy = (T[0][2] - T[2][0]) / S
		qz = (T[1][0] - T[0][1]) / S
	elif ( (T[0][0]>T[1][1]) and (T[0][0]>T[2][2]) ):
		S = math.sqrt(1.0+T[0][0]-T[1][1]-T[2][2]) * 2
		qw = (T[2][1] - T[1][2]) / S
		qx = 0.25 * S
		qy = (T[0][1] + T[1][0]) / S
		qz = (T[0][2] + T[2][0]) / S
	elif (T[1][1] > T[2][2]):
		S = math.sqrt(1.0+T[1][1]-T[0][0]-T[2][2]) * 2
		qw = (T[0][2] - T[2][0]) / S
		qx = (T[0][1] + T[1][0]) / S
		qy = 0.25 * S
		qz = (T[1][2] + T[2][1]) / S
	else:
		S = math.sqrt(1.0+T[2][2]-T[0][0]-T[1][1]) * 2
		qw = (T[1][0] - T[0][1]) / S
		qx = (T[0][2] + T[2][0]) / S
		qy = (T[1][2] + T[2][1]) / S
		qz = 0.25 * S


	return [qx, qy, qz, qw]

def printUpdateTruePoses(truePoses, clientId):
	print("updated poses after turn on the physics simulation")
	for truePose in truePoses:
		pos = list(p.getBasePositionAndOrientation(truePose.m, clientId)[0])
		quat = list(p.getBasePositionAndOrientation(truePose.m, clientId)[1])
		angles = p.getEulerFromQuaternion(quat)
		angles = [i*180/math.pi for i in angles]
		print( truePose.objName + ": " + str(pos) + ", " + str(angles) )

def read_print_poses(img_index):
	pose_info_path = "/home/rui/Documents/research/instance_segmentation/my_dataset/poses_info/" + img_index + ".txt"
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
		Objects[line[0]].append([float(line[4]), float(line[5]), float(line[6])]) ### put in the obj orient info
		temp_counter += 1
	for objName in Objects:
		print( objName + ": " + str(Objects[objName][1]) + ", " + str(Objects[objName][2]) )

	return Objects, targetObjectName

def printPoses(meshes):
	for mesh in meshes:
		print "hypo " + str(mesh.hypoIdx) + " " + str(mesh.pos) + " " + str(mesh.angles) + " " + \
							str(mesh.prob) + "\tfor object " + str(mesh.objIdx) + ": " + mesh.objName
	print "--------------------------------------\n"

def recordHypos(hypotheses, Objects, img_index):

	hypos_file = "/home/rui/Documents/research/instance_segmentation/my_dataset/hypos_info/" + img_index + ".txt"
	f_h = open(hypos_file, "w")

	for hypo in hypotheses:
		f_h.write(hypo.objName + " " + format(hypo.pos[0], '.3f') + " " + format(hypo.pos[1], '.3f') + " " + format(hypo.pos[2], '.3f') + " ")
		f_h.write(format(hypo.angles[0], '.3f') + " " + format(hypo.angles[1], '.3f') + " " + format(hypo.angles[2], '.3f') + " ")
		if (hypo.objName in Objects.keys()):
			delta_x = float(format(abs(hypo.pos[0] - Objects[hypo.objName][1][0]), '.3f'))
			delta_y = float(format(abs(hypo.pos[1] - Objects[hypo.objName][1][1]), '.3f'))
			delta_z = float(format(abs(hypo.pos[2] - Objects[hypo.objName][1][2]), '.3f'))
			delta_r = float(format(abs(hypo.angles[0] - Objects[hypo.objName][2][0]), '.3f'))
			delta_p = float(format(abs(hypo.angles[1] - Objects[hypo.objName][2][1]), '.3f'))
			delta_yaw = float(format(abs(hypo.angles[2] - Objects[hypo.objName][2][2]), '.3f'))
			if delta_r > 180.0:
				delta_r = 360.0 - delta_r
			if delta_p > 180.0:
				delta_p = 360.0 - delta_p
			if delta_yaw > 180.0:
				delta_yaw = 360.0 - delta_yaw
			f_h.write(str(delta_x) + " " + str(delta_y) + " " + str(delta_z) + " ")
			f_h.write(str(delta_r) + " " + str(delta_p) + " " + str(delta_yaw) + " ")
		f_h.write(format(hypo.prob, '.4f') + "\n")


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
		_m = p.createMultiBody(baseMass=massList[objName], baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
												basePosition=pos, baseOrientation=quat, physicsClientId=clientId)
		# _m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
		# 									basePosition=pos, baseOrientation=quat, physicsClientId=clientId)
		temp_mesh = TrueMesh(_m, objName, pos, quat, angles)
		truePoses.append(temp_mesh)

	p.setGravity(0.0, 0.0, -9.8, clientId)
	p.setRealTimeSimulation(1, clientId)

		
	print "Number of Objects in the ground truth (execution): " + str(len(truePoses))
	return truePoses, nObjectInExecuting

def createHypoMesh(currentlabelIdx, objIdx, meshFile, objName, InSceneProb, img_index, camera_extrinsic, clientId):
	
	hypotheses = []

	### first specify the collision (they are all the same to poses of the same object)
	_c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
			fileName=meshFile, meshScale=[1, 1, 1], physicsClientId=clientId)
	largest_prob = 0.0
	largest_prob_idx = currentlabelIdx

	### deal with scores first
	score_path = "../model_matching/examples/ycb/pose_candidates/" + str(img_index) + "/" + objName + "/best_pose_scores.txt"
	temp_scores = []
	f_scores = open(score_path)
	for line in f_scores:
		line = line.split()
		temp_scores.append(float(format(float(line[0]), '3f')))
	### convert the scores to probabilities based on existence probability
	score_sum = sum(temp_scores)
	for i in xrange(len(temp_scores)):
		temp_scores[i] = temp_scores[i] * InSceneProb / score_sum

	### Now deal with transforms (position + orientation)
	counter = 0
	scene_path = "../model_matching/examples/ycb/pose_candidates/" + str(img_index) + "/" + objName + "/best_pose_candidates.txt"
	f_transforms = open(scene_path)
	for line in f_transforms:
		temp_matrix = np.zeros(shape=(4,4))
		line = line.split()
		for i in xrange(len(line)):
			temp_matrix[i // 4][i % 4] = float(line[i])
		temp_matrix[3][3] = 1
		# print("temp_matrix: ", temp_matrix)
		temp_matrix = np.dot(camera_extrinsic, temp_matrix)
		### get position(x,y,z) and orientation(quaternion:qx,qy,qz,qw)
		temp_pos = [ temp_matrix[0][3], temp_matrix[1][3], temp_matrix[2][3] ]
		temp_quat = rotationMatrixToQuaternion(temp_matrix)
		temp_angles = p.getEulerFromQuaternion(temp_quat)
		temp_angles = [i * (180/math.pi) for i in temp_angles]

		### create visual shape (level of transparency reflects probability/confidence)
		_v = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=meshFile, meshScale=[1,1,1], 
									rgbaColor=[1.0, 1.0, 1.0, temp_scores[counter]], physicsClientId=clientId)
		### create multibody of the current hypothesis
		_m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v,
			basePosition=temp_pos, baseOrientation=temp_quat, physicsClientId=clientId)

		### update the most promosing hypothesis for the current object
		if temp_scores[counter] > largest_prob:
			largest_prob = temp_scores[counter]
			largest_prob_idx = currentlabelIdx + counter

		### add this hypothesis
		hypotheses.append( Mesh(_m, objName, objIdx, currentlabelIdx+counter, temp_pos, temp_quat, temp_angles, temp_scores[counter]) )
		counter += 1


	return hypotheses, largest_prob_idx


def planScene_generation(Objects, targetObjectName, img_index, camera_extrinsic, clientId):

	pose_candidates_path = "/home/rui/Documents/research/model_matching/examples/ycb/pose_candidates/" + str(img_index)
	hypotheses = []
	mostPromisingHypoIdxes = []
	currentlabelIdx = 0
	planningObjects = os.listdir(pose_candidates_path)
	nObjectInPlanning = len(planningObjects)

	### get the probability dict first
	prob_stats_path = "/home/rui/Documents/research/instance_segmentation/my_dataset/probability_statistics/iros20_ex/" + img_index + "_prob.txt";
	prob_dict = dict()
	f = open(prob_stats_path)
	for line in f:
		line = line.split()
		if line[0] in planningObjects:
			prob_dict[line[0]] = float(line[1])
	
	### first create hypotheses for the target object
	mm, pp = createHypoMesh(currentlabelIdx, 0, Objects[targetObjectName][0], targetObjectName, prob_dict[targetObjectName], img_index, camera_extrinsic, clientId)
	hypotheses += mm
	mostPromisingHypoIdxes.append(pp)
	currentlabelIdx = len(hypotheses)

	### Then deal with other objects than target object
	object_index = 1
	for objName in planningObjects:
		if objName == targetObjectName:
			continue
		if objName in Objects.keys() and objName != targetObjectName:
			mm, pp = createHypoMesh(currentlabelIdx, object_index, Objects[objName][0], objName, prob_dict[objName], img_index, camera_extrinsic, clientId)
		else:
			## these are phantom objects
			meshFile = "/mesh/"+ objName +"/google_16k/textured.obj"
			mm, pp = createHypoMesh(currentlabelIdx, object_index, meshFile, objName, prob_dict[objName], img_index, camera_extrinsic, clientId)
		hypotheses += mm
		mostPromisingHypoIdxes.append(pp)
		currentlabelIdx = len(hypotheses)
		object_index += 1
	print "Number of Objects in the planning scene: " + str(nObjectInPlanning)
	print "-------all hypotheses: -------"

	printPoses(hypotheses)


	return hypotheses, mostPromisingHypoIdxes, nObjectInPlanning
