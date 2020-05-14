from __future__ import division
import pybullet as p
import pybullet_data
import utils_example

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
	planningServer = p.connect(p.GUI)
	executingServer = p.connect(p.DIRECT)

	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	known_geometries_planning = []
	known_geometries_executing = []

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

	Objects, targetObjectName = utils_example.read_print_poses(img_index)
	print("targetObjectName", targetObjectName)
	tar_dim = target_objects_dim[targetObjectName]
	hypotheses, mostPromisingHypoIdxes, nObjectInPlanning = utils_example.planScene_generation(Objects, targetObjectName, img_index, camera_extrinsic, planningServer)

	################################################ work on goal configuration ######################################################
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
	goalPos_offset = [0.0, 0.0, tar_dim[2]/2+0.01]

	# goal_modes = ["multi", "single"]
	goal_mode = "multi"
	nHypos = 6
	for t_hp in xrange(nHypos):
		if goal_mode == "multi":
			MaxGoalsPerHypo = 2
			print "\n*******multi for Hypo " + str(t_hp) + "**********"
		# else:
		# 	MaxGoalsPerHypo = 4
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
				if goal_mode == "multi":
					print("\n*******multi: A new pose for Hypo " + str(t_hp) + "***************")
				else:
					print("\n*******single: A new pose for Hypo " + str(t_hp) + "***************")
				for j in range(1, 8):
					result_p = p.resetJointState(motomanID_p, j, home_configuration[j-1], physicsClientId=planningServer)
				for j in range(11, 18):
					result_p = p.resetJointState(motomanID_p, j, home_configuration[j-4], physicsClientId=planningServer)
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
				print("xyz_ee: " + str(xyz_ee))				
				### check collision for robot self and known obstacles
				isCollisionSelf = utils_example.collisionCheck_selfCollision(motomanID_p, planningServer)
				isCollisionKnownObs = utils_example.collisionCheck_knownObs(motomanID_p, known_geometries_planning, planningServer)
				if isCollisionSelf or isCollisionKnownObs:
					print "Collision with robot itself or known obstacles"
					temp_trials_pose += 1
					raw_input("Press Enter to continue")
					continue
				else:
					### check collision condition with all hypos of objects
					### (The opposite of the collision probability)
					collidedHypos = utils_example.collisionCheck_hypos(motomanID_p, hypotheses, planningServer)
					print "Collide with Hypos: " + str(collidedHypos)
					### compute the survivability
					if t_hp in collidedHypos:
						### the end effector collides with the pose it deems as the target pose
						temp_survival = 0.0
						print("it collide with the target object, and the survival: " + str(temp_survival))
						temp_trials_pose += 1
						raw_input("Press Enter to continue")
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
						print("the pose is valid, let's see the survival: " + str(temp_survival))
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
						print("temp_tar_hypos: ", temp_tar_hypos)
						print("temp_hyposForThatHypo: ", temp_hyposForThatHypo)

						temp_trials_pose += 1
						raw_input("Press Enter to continue")


	time.sleep(10000)