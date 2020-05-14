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


def calculateEuclidean(previous_state, current_state):
	tempSquare = 0.0
	for ii in xrange(len(previous_state)):
		tempSquare += math.pow(previous_state[ii]-current_state[ii] ,2)
	tempSquare = math.sqrt(tempSquare)
	return tempSquare


def trueScene_generation(Objects, clientId):

	# massList = {
	# 	"003_cracker_box": 2.8,
	# 	"004_sugar_box": 1.7,
	# 	"005_tomato_soup_can": 3.5,
	# 	"006_mustard_bottle": 1.9,
	# 	"008_pudding_box": 1.1,
	# 	"009_gelatin_box": 0.8,
	# 	"010_potted_meat_can": 2.8,
	# 	"011_banana": 1.0,
	# 	"019_pitcher_base": 1.6,
	# 	"021_bleach_cleanser": 2.7
	# }

	massList = {
		"003_cracker_box": 1.6,
		"004_sugar_box": 0.8,
		"005_tomato_soup_can": 2.2,
		"006_mustard_bottle": 0.8,
		"008_pudding_box": 1.1,
		"009_gelatin_box": 0.8,
		"010_potted_meat_can": 2.8,
		"011_banana": 1.0,
		"019_pitcher_base": 0.2,
		"021_bleach_cleanser": 1.4
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


def collisionCheck_truePoses(motomanID, truePoses, clientId):
	truePosesIdx = []
	### loop through truePoses
	for tp in truePoses:
		contacts = p.getContactPoints(motomanID, tp.m, physicsClientId=clientId)
		if len(contacts) != 0:
			### add the index of that true pose
			truePosesIdx.append(tp.objName)
	return truePosesIdx


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

def deleteTruePoses(truePoses, clientId):
	for trueobj in truePoses:
		p.removeBody(trueobj.m, clientId)

def execute_traj(img_index, home_configuration, motomanID, Objects, path, tar_dim, trajName, clientId):
	doIt = 1
	while (doIt == 1):
		truePoses, nObjectInExecuting = trueScene_generation(Objects, clientId)
		raw_input("Press to put Motoman to home configuration")
		### Put the motoman back to its home configuration
		for j in range(1, 8):
			result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
		for j in range(11, 18):
			result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
		message = "Press to execute " + trajName + " trajectory"
		raw_input(message)
		traj_file = path + "/" + trajName + "traj.txt"
		temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(traj_file, motomanID, truePoses, tar_dim, clientId)
		
		if trajName == "MSE":
			p.setGravity(0.0, 0.0, -1.8, clientId)
			p.setRealTimeSimulation(1, clientId)

		if img_index == "901" and trajName == "MCR":
			p.setGravity(0.0, 0.0, -1.8, clientId)
			p.setRealTimeSimulation(1, clientId)

		if img_index == "901" and trajName == "MLC":
			p.setGravity(0.0, 0.0, -1.8, clientId)
			p.setRealTimeSimulation(1, clientId)			
		
		raw_input("Press to put Motoman to home configuration\n")
		### Put the motoman back to its home configuration
		for j in range(1, 8):
			result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
		for j in range(11, 18):
			result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
		deleteTruePoses(truePoses, clientId)

		doIt = int(raw_input("do it again?"))


	return temp_ncollision, temp_isSuccess, temp_cost


# def executeAllTraj_example(home_configuration, motomanID, truePoses, path, tar_dim, clientId):


# 	raw_input("\nPress to put Motoman to home configuration")
# 	### Put the motoman back to its home configuration
# 	for j in range(1, 8):
# 		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
# 	for j in range(11, 18):
# 		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
# 	raw_input("Press to execute A star trajectory")
# 	astar_traj_file = path + "/Astartraj.txt"
# 	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(astar_traj_file, motomanID, truePoses, tar_dim, clientId)

# 	raw_input("\nPress to put Motoman to home configuration")
# 	### Put the motoman back to its home configuration
# 	for j in range(1, 8):
# 		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
# 	for j in range(11, 18):
# 		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
# 	raw_input("Press to execute MCR Greedy trajectory")
# 	mcrg_traj_file = path + "/MCRGtraj.txt"
# 	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mcrg_traj_file, motomanID, truePoses, tar_dim, clientId)


# 	raw_input("\nPress to put Motoman to home configuration")
# 	### Put the motoman back to its home configuration
# 	for j in range(1, 8):
# 		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
# 	for j in range(11, 18):
# 		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
# 	raw_input("Press to execute MCR exact trajectory")
# 	mcre_traj_file = path + "/MCREtraj.txt"
# 	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mcre_traj_file, motomanID, truePoses, tar_dim, clientId)


# 	raw_input("\nPress to put Motoman to home configuration")
# 	### Put the motoman back to its home configuration
# 	for j in range(1, 8):
# 		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
# 	for j in range(11, 18):
# 		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
# 	raw_input("Press to execute MaxSuccess greedy trajectory")
# 	msg_traj_file = path + "/MSGtraj.txt"
# 	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(msg_traj_file, motomanID, truePoses, tar_dim, clientId)


# 	raw_input("\nPress to put Motoman to home configuration")
# 	### Put the motoman back to its home configuration
# 	for j in range(1, 8):
# 		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
# 	for j in range(11, 18):
# 		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
# 	raw_input("Press to execute MaxSuccess exact trajectory")
# 	mse_traj_file = path + "/MSEtraj.txt"
# 	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mse_traj_file, motomanID, truePoses, tar_dim, clientId)
	

# 	raw_input("\nPress to put Motoman to home configuration")
# 	### Put the motoman back to its home configuration
# 	for j in range(1, 8):
# 		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
# 	for j in range(11, 18):
# 		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
# 	raw_input("Press to execute MCR-MLC trajectory")
# 	mcrmcg_traj_file = path + "/MCRMCGtraj.txt"
# 	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mcrmcg_traj_file, motomanID, truePoses, tar_dim, clientId)
	

# 	print "trajectories all executed and time record.\n\n"


def objects_union(currentList, newList):
	for obj in newList:
		if obj not in currentList:
			currentList.append(obj)

	return currentList