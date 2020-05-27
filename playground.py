from __future__ import division
import pybullet as p
import pybullet_data
import playground_utils

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

### indicate the id of the current scene you are working on
scene_index = sys.argv[1]
### create a folder to store all the images generated from the current scene
img_path = os.getcwd() + "/sensor_images/" + scene_index

if os.path.exists(img_path):
    shutil.rmtree(img_path)
os.mkdir(img_path)

### create two servers ###
### One for planning, the other executing (ground truth) ###
mode = sys.argv[2]
### experiment mode: use this when you perform large-scale experiment
### in this case, you don't want to see any GUI since they can slow your experiment
if mode == "e":
    planningServer = p.connect(p.DIRECT)
    executingServer = p.connect(p.DIRECT)
### visualization mode: use this when you only want to see 
### the visualization of the execution and the ground truth
elif mode == "v":
    planningServer = p.connect(p.DIRECT)
    executingServer = p.connect(p.GUI)
### planning mode: use this when you want to see how planning goes
elif mode == "p":
    planningServer = p.connect(p.GUI)
    executingServer = p.connect(p.DIRECT)

### set the real-time physics simulation ###
p.setGravity(0.0, 0.0, -9.8, executingServer)
p.setRealTimeSimulation(1, executingServer)

### add known geometries (i.e., robot, table)
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

############################### information related to Motoman arm (joint info) ###################################
### preserve the following five lines for debug purposes ###
# print "Motoman Robot: " + str(motomanID_p)
# num_joints = p.getNumJoints(motomanID_p, planningServer)
# print "Num of joints: " + str(num_joints)
# for i in range(num_joints):
#   print(p.getJointInfo(motomanID_p, i, planningServer))

### end-effector index
motoman_left_ee_idx = 10 ### left hand ee
motoman_right_ee_idx = 20 ### right hand ee

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
#####################################################################################################################

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
    aspect=1.78,
    nearVal=0.4,
    farVal=3.47)

#####################################################################################################

######### Let's put some objects on the table ##########
### first load in the ground truth of objects specified in a txt file
Objects, targetObjectName = playground_utils.read_print_poses(scene_index)
### generate these ground truth poses and report the number of objects
truePoses, nObjectInExecuting = playground_utils.trueScene_generation(Objects, planningServer)
### Take the initial images
playground_utils.sensorImageTaker(viewMatrix, projectionMatrix, executingServer, img_path)

################################# sense the initial scene ####################################################


# list of vacuum gripper's pick pose in object's frame
vacuum_gripper_localPicks = []
vacuum_gripper_localPicks.append([0, 0, -0.17, 0, 0, 0])
vacuum_gripper_localPicks.append([0, 0, 0.17, 0, 180, 0])
vacuum_gripper_localPicks.append([0, -0.15, 0, -90, 180, 0])
vacuum_gripper_localPicks.append([0, 0.15, 0, 90, 0, 0])
vacuum_gripper_localPicks.append([0.1, 0, 0, 0, -90, 0])
vacuum_gripper_localPicks.append([-0.1, 0, 0, 0, 90, 0])

# list of finger gripper's pick pose in object's frame

# initial object pose
# list of final object poses that work

#####################################################################################################
targetObj = truePoses[0]

vacuum_grasps = playground_utils.genVacuumGrasps(targetObj, vacuum_gripper_localPicks)
### try these vacuum grasps to see how they work
for vacuum_grasp in vacuum_grasps:
    goal_pose_pos = vacuum_grasp[0:3]
    goal_pose_quat = [vacuum_grasp[3], vacuum_grasp[4], vacuum_grasp[6], vacuum_grasp[5]]
    q_grasp = p.calculateInverseKinematics(bodyUniqueId=motomanID_p, endEffectorLinkIndex=motoman_left_ee_idx, 
                                        targetPosition=goal_pose_pos, targetOrientation=goal_pose_quat, 
                                        lowerLimits=ll, upperLimits=ul, jointRanges=jr, 
                                        maxNumIterations=20000, residualThreshold=0.0000001,
                                        physicsClientId=planningServer)
    for j in range(1, 8):
        result_p = p.resetJointState(motomanID_p, j, q_grasp[j-1], physicsClientId=planningServer)
    for j in range(11, 18):
        result_p = p.resetJointState(motomanID_p, j, q_grasp[j-4], physicsClientId=planningServer)
    p.stepSimulation(planningServer)
    raw_input("Enter to continue")




time.sleep(10000)