from __future__ import division
import pybullet as p
import pybullet_data
import utils_iros_final

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

### create two servers ###
### One for planning, the other executing (ground truth) ###
mode = sys.argv[3]
if mode == "e":
	planningServer = p.connect(p.DIRECT)
	executingServer = p.connect(p.DIRECT)
elif mode == "v":
	planningServer = p.connect(p.DIRECT)
	executingServer = p.connect(p.GUI)
elif mode == "p":
	planningServer = p.connect(p.GUI)
	executingServer = p.connect(p.DIRECT)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

### set the real-time physics simulation ###
# p.setGravity(0.0, 0.0, -9.8, executingServer)
# p.setRealTimeSimulation(1, executingServer)

known_geometries_planning = []
known_geometries_executing = []

### Introduce Motoman robot ###
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

nHypos = 5 ### Using pose clustering, let's return 5 pose representatives
nsamples = int(sys.argv[1]) ### We test it and find actually it is sufficient for 300 samples



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

#####################################################################################################

#####################################  object specification ##############################################
### currently try a simple scenario
### We may need later to have a function which can deploy objects for us
img_index = sys.argv[2]

### load in the ground truth
Objects, targetObjectName = utils_iros_final.read_print_poses(img_index)
# print("targetObjectName", targetObjectName)
# truePoses, nObjectInExecuting = utils_iros_final.trueScene_generation(Objects, executingServer)

# width, height, rgbImg, depthImg, segImg = p.getCameraImage(
# 	width=1280,
# 	height=720,
# 	viewMatrix=viewMatrix,
# 	projectionMatrix=projectionMatrix,
# 	physicsClientId=executingServer)


### load in the pose hypotheses
startTime = time.clock()
hypotheses, mostPromisingHypoIdxes, nObjectInPlanning = utils_iros_final.planScene_generation(Objects, targetObjectName, known_geometries_planning, img_index, camera_extrinsic, planningServer)
# print "Time elapsed to load the objects in the planning scene: ", time.clock() - startTime
# print "most promisings: " + str(mostPromisingHypoIdxes)
# print "--------------------------------------------------------------------------------------"
#############################################################################################################



#############################################################################################################
path = "./data"
### create a folder inside data indicating which image are you working on
data_path = path + "/" + img_index
if os.path.exists(data_path):
    shutil.rmtree(data_path)
os.mkdir(data_path)
### Now we can generate "labelWeights.txt" file
currentlabelWeightFile = data_path + "/labelWeights.txt"
f_labelWeights = open(currentlabelWeightFile, "w")
for hypo in hypotheses:
	f_labelWeights.write(str(hypo.hypoIdx) + " " + str(hypo.objIdx) + " " + str(hypo.prob) + "\n")
f_labelWeights.close()
### Now we can generate "mostPromisingLabels.txt" file
currentMostPromisingLabelsFile = data_path + "/mostPromisingLabels.txt"
f_mostPromisingLabels = open(currentMostPromisingLabelsFile, "w")
for mphi in mostPromisingHypoIdxes:
	f_mostPromisingLabels.write(str(mphi) + " ")
f_mostPromisingLabels.write("\n")
f_mostPromisingLabels.close()


n_roadmap = 5
for roadmap_idx in xrange(n_roadmap):
	print("roadmap " + str(roadmap_idx) + " for image " + img_index + "\n")
	### let's create the folder first
	roadmap_path = data_path + "/roadmap" + str(roadmap_idx)
	if os.path.exists(roadmap_path):
		shutil.rmtree(roadmap_path)
	os.mkdir(roadmap_path)
	#####################################specify start and goal####################################################
	startTime = time.clock()
	### specify q_start and set of q_goal first
	q_start = home_configuration
	### generate goal configurations ###
	goalSet = []
	goalHypos = [] ### Now it is a list of list
	survivalList = []
	# MaxGoalsPerHypo = 2
	MaxTrialsPerHypo = 7

	goalEuler = [0.0, math.pi, 0.0] ### overhand grasps
	target_objects_dim = {
		"009_gelatin_box": [0.07, 0.09, 0.026], 
		"008_pudding_box": [0.11, 0.09, 0.035],
		"010_potted_meat_can": [0.095, 0.056, 0.085]
	}
	tar_dim = target_objects_dim[targetObjectName]
	goalPos_offset = [0.0, 0.0, tar_dim[2]/2+0.03]


	goal_modes = ["multi", "single"]

	### for each target hypothesis
	for t_hp in xrange(nHypos):
		for goal_mode in goal_modes:
			if goal_mode == "multi":
				MaxGoalsPerHypo = 2
			# 	print "\n*******multi for Hypo " + str(t_hp) + "**********"
			else:
				MaxGoalsPerHypo = 4
			# 	print "\n*******single for Hypo" + str(t_hp) + "**********"
			temp_goalsForThatHypo = []
			temp_survivalForThatHypo = []
			temp_hyposForThatHypo = []
			### specify the position of the goal pose for that particular target hypothsis
			goal_pose_pos = []
			if goal_mode == "multi":
				for i in xrange(len(goalPos_offset)):
					goal_pose_pos.append(hypotheses[t_hp].pos[i] + goalPos_offset[i])

			temp_trials_pose = 0
			while temp_trials_pose < MaxTrialsPerHypo:
				# if goal_mode == "multi":
				# 	print("\n*******multi: A new pose for Hypo " + str(t_hp) + "***************")
				# else:
				# 	print("\n*******single: A new pose for Hypo " + str(t_hp) + "***************")
				for j in range(1, 8):
					result_p = p.resetJointState(motomanID_p, j, home_configuration[j-1], physicsClientId=planningServer)
				for j in range(11, 18):
					result_p = p.resetJointState(motomanID_p, j, home_configuration[j-4], physicsClientId=planningServer)
				if goal_mode == "single":
					if random.uniform(0, 1) < 0.5:
						random_offset_x = random.uniform(-tar_dim[0]/1.5, -tar_dim[0]/2.3)
					else:
						random_offset_x = random.uniform(tar_dim[0]/2.3, tar_dim[0]/1.5)
					if random.uniform(0, 1) < 0.5:
						random_offset_y = random.uniform(-tar_dim[1]/1.5, -tar_dim[1]/2.3)
					else:
						random_offset_y = random.uniform(tar_dim[1]/2.3, tar_dim[1]/1.5)
					goal_pose_pos = [hypotheses[t_hp].pos[0] + random_offset_x, hypotheses[t_hp].pos[1] + random_offset_y, hypotheses[t_hp].pos[2] + goalPos_offset[2]]
				# print "goal_pose_pos: " + str(goal_pose_pos)
				### specify the quaternion of that particular goal pose
				temp_goal_pose_quat = p.getQuaternionFromEuler([goalEuler[0], goalEuler[1], 
															goalEuler[2]+random.uniform(-math.pi, math.pi)])
				goal_pose_quat = [temp_goal_pose_quat[0], temp_goal_pose_quat[1], temp_goal_pose_quat[3], temp_goal_pose_quat[2]]
				### generate the IK for this pose of this hypotheses
				q_goal1 = p.calculateInverseKinematics(bodyUniqueId=motomanID_p, endEffectorLinkIndex=motoman_ee_idx, 
													targetPosition=goal_pose_pos, targetOrientation=goal_pose_quat, 
													lowerLimits=ll, upperLimits=ul, jointRanges=jr, 
													maxNumIterations=20000, residualThreshold=0.0000001,
													physicsClientId=planningServer)
				q_goal = []
				for bb in xrange(len(home_configuration)):
					q_goal.append(float(format(q_goal1[bb], '.2f')))
				# print("q_goal: ", q_goal)
				for j in range(1, 8):
					result_p = p.resetJointState(motomanID_p, j, q_goal[j-1], physicsClientId=planningServer)
				for j in range(11, 18):
					result_p = p.resetJointState(motomanID_p, j, q_goal[j-4], physicsClientId=planningServer)
				p.stepSimulation(planningServer)
				xyz_ee = p.getLinkState(bodyUniqueId=motomanID_p, linkIndex=10, computeForwardKinematics=True, physicsClientId=planningServer)[0]
				# print("xyz_ee: " + str(xyz_ee))
				### check collision for robot self and known obstacles
				isCollisionSelf = utils_iros_final.collisionCheck_selfCollision(motomanID_p, planningServer)
				isCollisionKnownObs = utils_iros_final.collisionCheck_knownObs(motomanID_p, known_geometries_planning, planningServer)
				if isCollisionSelf or isCollisionKnownObs:
					# print "Collision with robot itself or known obstacles"
					temp_trials_pose += 1
					# raw_input("Press Enter to continue")
					continue
				else:
					### check collision condition with all hypos of objects
					### (The opposite of the collision probability)
					collidedHypos = utils_iros_final.collisionCheck_hypos(motomanID_p, hypotheses, planningServer)
					# print "Collide with Hypos: " + str(collidedHypos)
					### compute the survivability
					if t_hp in collidedHypos:
						### the end effector collides with the pose it deems as the target pose
						temp_survival = 0.0
						# print("it collide with the target object, and the survival: " + str(temp_survival))
						temp_trials_pose += 1
						# raw_input("Press Enter to continue")
						continue
					else:
						temp_tar_hypos = [t_hp]
						temp_survival = 1.0
						collisionPerObj = [0.0] * nObjectInPlanning
						for ch in collidedHypos:
							if hypotheses[ch].objIdx != 0:
								collisionPerObj[hypotheses[ch].objIdx] += hypotheses[ch].prob
						for cpobs in collisionPerObj:
							temp_survival *= (1 - cpobs)
						# print("the pose is valid, let's see the survival: " + str(temp_survival))
						### add the q_goals and its corresponding survivability, add its identity
						temp_goalsForThatHypo.append(q_goal)
						temp_survivalForThatHypo.append(temp_survival)
						### Check if it is good for other target hypotheses
						for tar_hp in xrange(nHypos):
							if tar_hp == t_hp or tar_hp in collidedHypos:
								### no duplicate or collision allowed
								continue
							else:
								### Now check if tar_hp is a good hypo as well
								if (abs(xyz_ee[0] - hypotheses[tar_hp].pos[0]) < tar_dim[0]/2) and (abs(xyz_ee[1] - hypotheses[tar_hp].pos[1]) < tar_dim[1]/2):
									temp_tar_hypos.append(tar_hp)
						temp_hyposForThatHypo.append(temp_tar_hypos)
						# print("temp_tar_hypos: ", temp_tar_hypos)
						# print("temp_hyposForThatHypo: ", temp_hyposForThatHypo)

						temp_trials_pose += 1
						# raw_input("Press Enter to continue")
			
		
			### You are here since you finish generating poses for a particular mode (multi or single)
			### sort temp_survivalForThatHypo and pick top (MaxGoalsPerHypo) ones
			idx_rank = sorted( range(len(temp_survivalForThatHypo)), key=lambda k: temp_survivalForThatHypo[k], reverse=True )
			if len(idx_rank) >= MaxGoalsPerHypo:
				### add top (MaxGoalsPerHypo) ones
				for mm in xrange(MaxGoalsPerHypo):
					goalSet.append(temp_goalsForThatHypo[idx_rank[mm]])
					goalHypos.append(temp_hyposForThatHypo[idx_rank[mm]])
					survivalList.append(temp_survivalForThatHypo[idx_rank[mm]])
			else:
				goalSet += temp_goalsForThatHypo
				goalHypos += temp_hyposForThatHypo
				survivalList += temp_survivalForThatHypo


	# print("goalSet:", goalSet)
	# print('goalHypos: ', goalHypos)
	# print("survivalList: ", survivalList)
	# print("Finish goal generation\n")

	### Let's store these goals 
	goalsFile = roadmap_path + "/goals.txt"
	f_goals = open(goalsFile, "w")
	for i in xrange(len(goalSet)):
		g_hypos = goalHypos[i]
		f_goals.write(str(nsamples+i+1))
		for gh in g_hypos:
			f_goals.write(" " + str(gh))
		f_goals.write("\n")
	f_goals.close()
	########################################################################################################################


	##############################################roadmap generation########################################################
	startTime = time.clock()
	# print("start sampling")
	### sampling limits
	x_ll = motomanBasePosition[0] - 0.35
	x_ul = tablePosition[0] + table_dim[0]/2
	# y_ll = -table_dim[1]/2
	y_ll = motomanBasePosition[1] -0.05
	y_ul = tablePosition[1] + table_dim[1]/2
	z_ll = tablePosition[2] + table_dim[2]/2
	z_ul = z_ll + 0.6
	############# start sampling ##############
	currentSamplesFile = roadmap_path + "/samples.txt"
	f_samples = open(currentSamplesFile, "w")
	nodes = []
	temp_counter = 0

	while temp_counter < nsamples:
		### sample a cartesian ee pose and calculate the IK solution
		if temp_counter < int(nsamples*0.01):
			temp_x = float(format(random.uniform(x_ll, x_ul), '.2f'))
			temp_y = float(format(random.uniform(y_ll, y_ul), '.2f'))
			temp_z = float(format(random.uniform(z_ll, z_ul), '.2f'))
			ikSolution1 = p.calculateInverseKinematics(bodyUniqueId=motomanID_p, endEffectorLinkIndex=motoman_ee_idx,
												targetPosition=[temp_x, temp_y, temp_z], 
												lowerLimits=ll, upperLimits=ul, jointRanges=jr,
												physicsClientId=planningServer)

			ikSolution = []
			for nn in xrange(len(home_configuration)):
				ikSolution.append(float(format(ikSolution1[nn], '.2f')))
		else:
			temp_j0 = float(format(random.uniform(ll[0], ul[0]), '.2f'))
			temp_j1 = float(format(random.uniform(ll[1], ul[1]), '.2f'))
			temp_j2 = float(format(random.uniform(ll[2], ul[2]), '.2f'))
			temp_j3 = float(format(random.uniform(ll[3], ul[3]), '.2f'))
			temp_j4 = float(format(random.uniform(ll[4], ul[4]), '.2f'))
			temp_j5 = float(format(random.uniform(ll[5], ul[5]), '.2f'))
			temp_j6 = float(format(random.uniform(ll[6], ul[6]), '.2f'))
			temp_j7 = 0.0
			temp_j8 = 0.0
			temp_j9 = 0.0
			temp_j10 = 0.0
			temp_j11 = 0.0
			temp_j12 = 0.0
			temp_j13 = 0.0
			ikSolution = [temp_j0, temp_j1, temp_j2, temp_j3, temp_j4, temp_j5, temp_j6, temp_j7, temp_j8, temp_j9, temp_j10, temp_j11, temp_j12, temp_j13]
		for j in range(1, 8):
			result_p = p.resetJointState(motomanID_p, j, ikSolution[j-1], physicsClientId=planningServer)
		for j in range(11, 18):
			result_p = p.resetJointState(motomanID_p, j, ikSolution[j-4], physicsClientId=planningServer)
		p.stepSimulation(planningServer)
		### check collision for robot self and known obstacles
		isCollisionSelf = utils_iros_final.collisionCheck_selfCollision(motomanID_p, planningServer)
		isCollisionKnownObs = utils_iros_final.collisionCheck_knownObs(motomanID_p, known_geometries_planning, planningServer)
		if (not isCollisionSelf) and (not isCollisionKnownObs):
			nodes.append(ikSolution)
			### write it into a sample file
			f_samples.write(str(temp_counter) + " " + str(ikSolution[0]) + " " + str(ikSolution[1]) + " " \
				+ str(ikSolution[2]) + " " + str(ikSolution[3]) + " " + str(ikSolution[4]) + " " \
				+ str(ikSolution[5]) + " " + str(ikSolution[6]) + "\n")
			temp_counter += 1
	# print("Total time for sampling: " + str(time.clock()-startTime) + " seconds.\n")


	############## connect neighbors to build roadmaps #############
	startTime = time.clock()
	connectivity = np.zeros((nsamples, nsamples))
	tree = spatial.KDTree(nodes)
	neighbors_const = 1.5 * math.e * (1 + 1/(len(home_configuration)/2))
	num_neighbors = int(neighbors_const * math.log(nsamples))
	if num_neighbors >= nsamples:
		num_neighbors = nsamples - 1
	print "num_neighbors: " + str(num_neighbors)
	currentRoadmapFile = roadmap_path + "/roadmap.txt"
	f_roadmap = open(currentRoadmapFile, "w")
	### for each node
	for i in xrange(len(nodes)):
		queryNode = nodes[i]
		knn = tree.query(queryNode, k=num_neighbors, p=2)
		### for each neighbor
		for j in xrange(len(knn[1])):
			if knn[1][j] == i or connectivity[i][knn[1][j]] == 1:
				### if the neighbor is the query node itself
				### or the connectivity has been checked before
				### then skip the edge checking procedure
				continue
			### Otherwise, check the edge validity (in terms of collision with robot itself and known obstacles)
			### between the query node and the current neighbor
			neighbor = nodes[knn[1][j]]
			isEdgeValid = utils_iros_final.checkEdgeValidity(queryNode, neighbor, motomanID_p,
														known_geometries_planning, planningServer)
			if isEdgeValid:
				### write this edge information with their costs and labels into the txt file
				### It is a valid edge in terms of no collision with robot itself and known obstacles
				### Let's check the collision status for each hypothesis for the purpose of labeling
				temp_labels = utils_iros_final.label_the_edge(queryNode, neighbor, motomanID_p, hypotheses, planningServer)
				f_roadmap.write(str(i) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f') + " ")
				for tl in temp_labels:
					f_roadmap.write(str(tl) + " ")
				f_roadmap.write("\n")
				### update connectivity information
				connectivity[i][knn[1][j]] = 1
				connectivity[knn[1][j]][i] = 1

		if i % 100 == 99:
			print "finish labeling and connecting neighbors for node " + str(i)
	print "finish all the neighbors"
	print "Time elapsed: ", time.clock() - startTime

	########################################################


	############## add the start node and goal nodes to the roadmap ##########
	startConnectSuccess = False
	startTime = time.clock()
	nodes.append(q_start)
	f_samples.write(str(temp_counter) + " " + str(q_start[0]) + " " + str(q_start[1]) + " " \
		+ str(q_start[2]) + " " + str(q_start[3]) + " " + str(q_start[4]) + " " + \
		str(q_start[5]) + " " + str(q_start[6]) + "\n")
	temp_counter += 1
	### connect the start to the roadmap
	tree = spatial.KDTree(nodes)
	queryNode = nodes[temp_counter-1]
	knn = tree.query(queryNode, k=num_neighbors, p=2)

	### for each neighbor
	for j in xrange(len(knn[1])):
		if knn[1][j] == (temp_counter-1):
			continue
		else:
			### check collision
			neighbor = nodes[knn[1][j]]
			isEdgeValid = utils_iros_final.checkEdgeValidity(queryNode, neighbor, motomanID_p,
															known_geometries_planning, planningServer)
			if isEdgeValid:
				### examine the survivability of the edge
				### first get the labels 
				temp_labels = utils_iros_final.label_the_edge(queryNode, neighbor, motomanID_p, hypotheses, planningServer)
				### compute the survivability
				temp_survival = 1.0
				collisionPerObj = [0.0] * nObjectInPlanning
				for tl in temp_labels:
					collisionPerObj[hypotheses[tl].objIdx] += hypotheses[tl].prob
				for cpobs in collisionPerObj:
					temp_survival *= (1 - cpobs)
				if temp_survival > 0.4:
					f_roadmap.write(str(temp_counter-1) + " " + str(knn[1][j]) + " " \
						+ format(knn[0][j], '.4f') + " ")
					for l in temp_labels:
						f_roadmap.write(str(l) + " ")
					f_roadmap.write("\n")
					startConnectSuccess = True
	if startConnectSuccess:
		print("connect the start to the roadmap")	

	### loop through goalSet
	goalConnectSuccess = False

	for i in xrange(len(goalSet)):
		q_goal = goalSet[i]
		nodes.append(q_goal)
		f_samples.write(str(temp_counter) + " " + str(q_goal[0]) + " " + str(q_goal[1]) + " " \
			+ str(q_goal[2]) + " " + str(q_goal[3]) + " " + str(q_goal[4]) + " " + \
			str(q_goal[5]) + " " + str(q_goal[6]) + "\n")

		### connect the goal to the roadmap
		tree = spatial.KDTree(nodes)
		queryNode = nodes[temp_counter]
		knn = tree.query(queryNode, k=num_neighbors, p=2)
		### for each neighbor
		for j in xrange(len(knn[1])):
			if knn[1][j] == temp_counter:
				continue
			else:
				### check collision
				neighbor = nodes[knn[1][j]]
				isEdgeValid = utils_iros_final.checkEdgeValidity(queryNode, neighbor, motomanID_p, 
																	known_geometries_planning, planningServer)
				if isEdgeValid:
					### examine the survivability of the edge
					### first get the labels				
					temp_labels = utils_iros_final.label_the_edge(queryNode, neighbor, motomanID_p, hypotheses, planningServer)
					### compute the survivability
					temp_survival = 1.0
					collisionPerObj = [0.0] * nObjectInPlanning
					for tl in temp_labels:
						collisionPerObj[hypotheses[tl].objIdx] += hypotheses[tl].prob
					for cpobs in collisionPerObj:
						temp_survival *= (1 - cpobs)
					if temp_survival > 0.2:
						f_roadmap.write(str(temp_counter) + " " + str(knn[1][j]) + " " \
							+ format(knn[0][j], '.4f') + " ")
						for l in temp_labels:
							f_roadmap.write(str(l) + " ")
						f_roadmap.write("\n")
						goalConnectSuccess = True
		temp_counter += 1

	if goalConnectSuccess == True:
		print("connect goals to the roadmap")
	f_samples.close()
	f_roadmap.close()
	print "query for start and goals in " + str(time.clock() - startTime) + " second."
	# print len(nodes)
	######################################################################################################


	############ call planning algorithms ################
	print "start planning..."
	executeFile = "../planning_iros_20/main_iros" + " " + str(sys.argv[1]) + " " + str(sys.argv[2]) + " " + str(roadmap_idx)
	subprocess.call(executeFile, shell=True)

	truePoses, nObjectInExecuting = utils_iros_final.trueScene_generation(Objects, executingServer)

	width, height, rgbImg, depthImg, segImg = p.getCameraImage(
		width=1280,
		height=720,
		viewMatrix=viewMatrix,
		projectionMatrix=projectionMatrix,
		physicsClientId=executingServer)


	utils_iros_final.executeAllTraj_example(home_configuration, motomanID_e, truePoses, roadmap_path, tar_dim, executingServer)
	utils_iros_final.deleteTruePoses(truePoses, executingServer)
	for j in range(1, 8):
		result_p = p.resetJointState(motomanID_p, j, home_configuration[j-1], physicsClientId=executingServer)
	for j in range(11, 18):
		result_p = p.resetJointState(motomanID_p, j, home_configuration[j-4], physicsClientId=executingServer)


### Before leave, compute the average statistics for the current image
### (average: #collisions, #success, path cost and time)
total_collision = [0, 0, 0, 0, 0, 0]
total_success = [0, 0, 0, 0, 0, 0]
total_pathCost = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
total_time = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
for i in xrange(n_roadmap):
	temp_statFile = data_path + "/roadmap" + str(i) + "/statistics.txt"
	temp_time = data_path + "/roadmap" + str(i) + "/times.txt"
	
	f_tempstat = open(temp_statFile)
	counter_tempstat = 0
	for line in f_tempstat:
		line = line.split()
		total_collision[counter_tempstat] += int(line[0])
		total_success[counter_tempstat] += int(line[1])
		total_pathCost[counter_tempstat] += float(format(float(line[2]), '.3f'))
		counter_tempstat += 1
	f_tempstat.close()

	f_temptime = open(temp_time)
	counter_time = 0
	for line in f_temptime:
		line = line.split()
		total_time[counter_time] += float(format(float(line[0]), '.3f'))
		counter_time += 1
	f_temptime.close()


image_statistics_file = data_path + "/image_statistics.txt"
f_imageStat = open(image_statistics_file, "w")
total_collision = [i / n_roadmap for i in total_collision]
total_success = [i / n_roadmap for i in total_success]
total_pathCost = [i / n_roadmap for i in total_pathCost]
total_time = [i / n_roadmap for i in total_time]
for j in xrange(len(total_collision)):
	f_imageStat.write(str(total_collision[j]) + " " + str(total_success[j]) + " " + str(total_pathCost[j]) + " " + str(total_time[j]) + "\n") 

print("Finish the current image")
print("++++++++++++++++++++++++++++\n\n")
