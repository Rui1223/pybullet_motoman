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
	# for objName in Objects:
	# 	print( objName + ": " + str(Objects[objName][1]) + ", " + str(Objects[objName][2]) )

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
		# 										basePosition=pos, baseOrientation=quat, physicsClientId=clientId)
		_m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
											basePosition=pos, baseOrientation=quat, physicsClientId=clientId)
		temp_mesh = TrueMesh(_m, objName, pos, quat, angles)
		truePoses.append(temp_mesh)

	# p.setGravity(0.0, 0.0, -9.8, clientId)
	# p.setRealTimeSimulation(1, clientId)

	print "Number of Objects in the ground truth (execution): " + str(len(truePoses))
	return truePoses, nObjectInExecuting


def createHypoMesh(currentlabelIdx, objIdx, meshFile, objName, InSceneProb, img_index, known_geometries, camera_extrinsic, clientId):
	
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


def planScene_generation(Objects, targetObjectName, known_geometries, img_index, camera_extrinsic, clientId):

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
	mm, pp = createHypoMesh(currentlabelIdx, 0, Objects[targetObjectName][0], targetObjectName, prob_dict[targetObjectName], img_index, known_geometries, camera_extrinsic, clientId)
	hypotheses += mm
	mostPromisingHypoIdxes.append(pp)
	currentlabelIdx = len(hypotheses)

	### Then deal with other objects than target object
	object_index = 1
	for objName in planningObjects:
		if objName == targetObjectName:
			continue
		if objName in Objects.keys() and objName != targetObjectName:
			mm, pp = createHypoMesh(currentlabelIdx, object_index, Objects[objName][0], objName, prob_dict[objName], img_index, known_geometries, camera_extrinsic, clientId)
		else:
			## these are phantom objects
			meshFile = "/mesh/"+ objName +"/google_16k/textured.obj"
			mm, pp = createHypoMesh(currentlabelIdx, object_index, meshFile, objName, prob_dict[objName], img_index, known_geometries, camera_extrinsic, clientId)
		hypotheses += mm
		mostPromisingHypoIdxes.append(pp)
		currentlabelIdx = len(hypotheses)
		object_index += 1
	print "Number of Objects in the planning scene: " + str(nObjectInPlanning)
	print "-------all hypotheses: -------"

	# printPoses(hypotheses)


	return hypotheses, mostPromisingHypoIdxes, nObjectInPlanning


def collisionCheck_amongObjects(m, known_geometries, benchmarkType, clientId):
	isCollision = False
	for g in known_geometries:
		if benchmarkType == "table" and str(g) == "2":
			continue
		if benchmarkType == "shelf" and str(g) == "1":
			continue
		if benchmarkType == "shelf" and str(g) == "5":
			continue
		contacts = p.getClosestPoints(m, g, 0.01, physicsClientId=clientId)
		if len(contacts) != 0:
			isCollision = True
			break

	return isCollision

def collisionCheck_selfCollision(motomanID, clientId):
	isCollision = False
	contacts = p.getContactPoints(bodyA=motomanID, bodyB=motomanID, physicsClientId=clientId)
	if len(contacts) != 0:
		isCollision = True
		# print "robot self collision occurs!"
		# print contacts
	return isCollision

def collisionCheck_knownObs(motomanID, known_geometries, clientId):
	isCollision = False
	### loop through all known obstacles
	for g in known_geometries:
		if g == 0: ## no need to change robot self-collision again
			continue
		contacts = p.getContactPoints(motomanID, g, physicsClientId=clientId)
		if len(contacts) != 0:
			isCollision = True
			# print "collision with known obstacle " + str(g)
			break
	return isCollision

def collisionCheck_hypos(motomanID, hypotheses, clientId):
	collidedHypos = []
	### loop through all hypotheses
	for hp in hypotheses:
		contacts = p.getContactPoints(motomanID, hp.m, physicsClientId=clientId)
		if len(contacts) != 0:
			### add the index of that hypo
			collidedHypos.append(hp.hypoIdx)

	return collidedHypos

def checkEdgeValidity(n1, n2, motomanID, known_geometries, clientId):
	# max_step = 8 * math.pi / 180
	# max_diff = max(math.fabs(n1[0]-n2[0]), math.fabs(n1[1]-n2[1]), 
	# 		math.fabs(n1[2]-n2[2]), math.fabs(n1[3]-n2[3]), math.fabs(n1[4]-n2[4]), 
	# 		math.fabs(n1[5]-n2[5]), math.fabs(n1[6]-n2[6]))
	# print("max_diff: " + str(max_diff))
	# nseg = int(max_diff / max_step)
	# print("nseg: " + str(nseg))

	nseg = 5

	if nseg == 0:
		nseg = 1
	isEdgeValid = True
	for i in xrange(1, nseg):
		interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
		interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
		interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
		interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
		interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
		interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
		interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
		intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]	
		for j in range(1, 8):
			result = p.resetJointState(motomanID, j, intermNode[j-1], physicsClientId=clientId)
		for j in range(11, 18):
			result = p.resetJointState(motomanID, j, 0.0, physicsClientId=clientId)
		p.stepSimulation(clientId)
		isCollisionSelf = collisionCheck_selfCollision(motomanID, clientId)
		isCollisionKnownObs = collisionCheck_knownObs(motomanID, known_geometries, clientId)

		if isCollisionSelf or isCollisionKnownObs:
			isEdgeValid = False
			break

	return isEdgeValid

def label_the_edge(n1, n2, motomanID, hypotheses, clientId):
	temp_labels = []
	labels_status = [False] * len(hypotheses)

	nseg = 10

	if nseg == 0:
		nseg = 1
	for i in xrange(0, nseg+1):
		interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
		interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
		interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
		interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
		interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
		interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
		interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
		intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
		for j in range(1, 8):
			result = p.resetJointState(motomanID, j, intermNode[j-1], physicsClientId=clientId)
		for j in range(11, 18):
			result = p.resetJointState(motomanID, j, 0.0, physicsClientId=clientId)
		p.stepSimulation(clientId)
		for hp in hypotheses:
			### Before you do collision checker
			### check if that hypo has been checked before
			if labels_status[hp.hypoIdx] == False:
				contacts = p.getContactPoints(motomanID, hp.m, physicsClientId=clientId)
				if len(contacts) != 0:
					temp_labels.append(hp.hypoIdx)
					labels_status[hp.hypoIdx] = True

	return temp_labels


def collisionCheck_truePoses(motomanID, truePoses, clientId):
	truePosesIdx = []
	### loop through truePoses
	for tp in truePoses:
		contacts = p.getContactPoints(motomanID, tp.m, physicsClientId=clientId)
		if len(contacts) != 0:
			### add the index of that true pose
			truePosesIdx.append(tp.objName)
	return truePosesIdx



def printPoses(meshes):
	for mesh in meshes:
		print "hypo " + str(mesh.hypoIdx) + " " + str(mesh.pos) + " " + str(mesh.angles) + " " + \
							str(mesh.prob) + "\tfor object " + str(mesh.objIdx) + ": " + mesh.objName
	print "--------------------------------------\n"

def calculateEuclidean(previous_state, current_state):
	tempSquare = 0.0
	for ii in xrange(len(previous_state)):
		tempSquare += math.pow(previous_state[ii]-current_state[ii] ,2)
	tempSquare = math.sqrt(tempSquare)
	return tempSquare

def local_move(n1, n2, motomanID, truePoses, clientId):
	local_poseIdx = []
	# step = 3.0 * math.pi / 180
	# nseg = int(math.ceil(max(math.fabs(n1[0]-n2[0]), 
	# 	math.fabs(n1[1]-n2[1]), math.fabs(n1[2]-n2[2]), 
	# 	math.fabs(n1[3]-n2[3]), math.fabs(n1[4]-n2[4]), 
	# 	math.fabs(n1[5]-n2[5]), math.fabs(n1[6]-n2[6]))) / step)
	# if nseg == 0:
	# 	nseg = 1
	nseg = 25
	for i in xrange(0, nseg+1):
		interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
		interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
		interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
		interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
		interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
		interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
		interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
		intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
		for j in range(1, 8):
			result = p.resetJointState(motomanID, j, intermNode[j-1], physicsClientId=clientId)
		for j in range(11, 18):
			result = p.resetJointState(motomanID, j, 0.0, physicsClientId=clientId)
		p.stepSimulation(clientId)
		# xyz_ee = p.getLinkState(bodyUniqueId=motomanID, linkIndex=10, computeForwardKinematics=True, physicsClientId=clientId)[0]
		# print("xyz_ee: " + str(xyz_ee))
		## add collision checker
		temp_poseIdx = collisionCheck_truePoses(motomanID, truePoses, clientId)
		local_poseIdx = objects_union(local_poseIdx, temp_poseIdx)

		time.sleep(0.05)
	return local_poseIdx

def executeTrajectory(traj_file, motomanID, truePoses, tar_dim, clientId):
	traj = []
	f = open(traj_file)
	previous_state = None
	### Meanwhile compute the cost as well
	trajectoryCost = 0.0
	trajectoryCollision = []
	isSuccess = 1

	for line in f:
		current_state = line.split()
		current_state = map(float, current_state)
		if (previous_state is not None):
			trajectoryCost += calculateEuclidean(previous_state, current_state)
			trajectoryCollision = objects_union(trajectoryCollision, 
											local_move(previous_state, current_state, motomanID, truePoses, clientId))
		previous_state = current_state
	print "collisions: " + str(trajectoryCollision) + ",  total: " + str(len(trajectoryCollision))
	if len(trajectoryCollision) != 0:
		isSuccess = 0 ### collision

	### now work on success evaluation
	### suppose we limit our target to gelatin box and pudding box
	xyz_ee = p.getLinkState(bodyUniqueId=motomanID, linkIndex=10, computeForwardKinematics=True, physicsClientId=clientId)[0]
	print("xyz_ee: " + str(xyz_ee))
	print("target pos: " + str(truePoses[0].pos))

	if (abs(xyz_ee[0]-truePoses[0].pos[0]) >= tar_dim[0]/2) or (abs(xyz_ee[1]-truePoses[0].pos[1]) >= tar_dim[1]/2):
		isSuccess = 0 ### deviation


	print("isSuccess: " + str(isSuccess))
	print("trajectoryCost: " + str(trajectoryCost))
	return len(trajectoryCollision), isSuccess, trajectoryCost


def executeAllTraj_example(home_configuration, motomanID, truePoses, path, tar_dim, clientId):
	statistics_file = path + "/statistics.txt"
	### write in (#obs, success, cost) information for each search method
	f = open(statistics_file, "w")

	# raw_input("\nPress to put Motoman to home configuration")
	# print("\nPress to put Motoman to home configuration")
	### Put the motoman back to its home configuration
	for j in range(1, 8):
		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
	for j in range(11, 18):
		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
	# raw_input("Press to execute A star trajectory")
	print("\nPress to execute A star trajectory")
	astar_traj_file = path + "/Astartraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(astar_traj_file, motomanID, truePoses, tar_dim, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )

	# raw_input("\nPress to put Motoman to home configuration")
	# print("\nPress to put Motoman to home configuration")
	### Put the motoman back to its home configuration
	for j in range(1, 8):
		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
	for j in range(11, 18):
		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
	# raw_input("Press to execute MCR Greedy trajectory")
	print("\nPress to execute MCR Greedy trajectory")
	mcrg_traj_file = path + "/MCRGtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mcrg_traj_file, motomanID, truePoses, tar_dim, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )

	# raw_input("\nPress to put Motoman to home configuration")
	# print("\nPress to put Motoman to home configuration")
	### Put the motoman back to its home configuration
	for j in range(1, 8):
		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
	for j in range(11, 18):
		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
	# raw_input("Press to execute MCR exact trajectory")
	print("\nPress to execute MCR exact trajectory")
	mcre_traj_file = path + "/MCREtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mcre_traj_file, motomanID, truePoses, tar_dim, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )

	
	# raw_input("\nPress to put Motoman to home configuration")
	# print("\nPress to put Motoman to home configuration")
	### Put the motoman back to its home configuration
	for j in range(1, 8):
		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
	for j in range(11, 18):
		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
	# raw_input("Press to execute MaxSuccess greedy trajectory")
	print("\nPress to execute MaxSuccess greedy trajectory")
	msg_traj_file = path + "/MSGtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(msg_traj_file, motomanID, truePoses, tar_dim, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )


	# raw_input("\nPress to put Motoman to home configuration")
	# print("\nPress to put Motoman to home configuration")
	### Put the motoman back to its home configuration
	for j in range(1, 8):
		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
	for j in range(11, 18):
		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
	# raw_input("Press to execute MaxSuccess exact trajectory")
	print("\nPress to execute MaxSuccess exact trajectory")
	mse_traj_file = path + "/MSEtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mse_traj_file, motomanID, truePoses, tar_dim, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )	

	# raw_input("\nPress to put Motoman to home configuration")
	# print("\nPress to put Motoman to home configuration")
	### Put the motoman back to its home configuration
	for j in range(1, 8):
		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
	for j in range(11, 18):
		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
	# raw_input("Press to execute MCR-MLC trajectory")
	print("\nPress to execute MCR-MLC trajectory")
	mcrmcg_traj_file = path + "/MCRMCGtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mcrmcg_traj_file, motomanID, truePoses, tar_dim, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )
	

	f.close()
	print "\ntrajectories all executed and time record.\n\n"


def deleteTruePoses(truePoses, clientId):
	for trueobj in truePoses:
		p.removeBody(trueobj.m, clientId)

def objects_union(currentList, newList):
	for obj in newList:
		if obj not in currentList:
			currentList.append(obj)

	return currentList