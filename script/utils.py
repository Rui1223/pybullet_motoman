### utils file contains some of the common functionalities
### shared by lots of experimental setting

from __future__ import division
import pybullet_utils.bullet_client as bc
import pybullet as p
import pybullet_data

from collections import OrderedDict
import os
import random
import math
import numpy as np
import time
import IPython


def norm2(v):
    norm = 0.0
    for i in range(len(v)):
        norm += v[i]*v[i]
    norm = math.sqrt(norm)
    return norm


def calculateInnerProduct(v1, v2):
    ip = 0.0
    for i in range(len(v1)):
        ip += v1[i]*v2[i]
    return ip


def calculateNorm2(v1, v2):
    temp_dist = 0.0
    for i in range(len(v1)):
        temp_dist += (v1[i] - v2[i])**2
    temp_dist = math.sqrt(temp_dist)
    return temp_dist


def computePoseDist_pos(pose1, pose2):
    ### Input: format for pose [x,y,z]
    pose_dist = calculateNorm2(pose1, pose2)
    return pose_dist


def computePoseDist_quat(quat1, quat2):
    lamb = calculateInnerProduct(quat1, quat2)
    return (1 - abs(lamb))


def interpolatePosition(P1, P2, f):
    ### Here pos P: [x,y,z]
    x = P1[0] + (P2[0]-P1[0]) * f
    y = P1[1] + (P2[1]-P1[1]) * f
    z = P1[2] + (P2[2]-P1[2]) * f
    P = [x, y, z]
    return P


def interpolateQuaternion(Q1, Q2, f):
    epsilon = 0.1
    ### Here quat Q: [x,y,z,w]
    x1 = Q1[0]
    y1 = Q1[1]
    z1 = Q1[2]
    w1 = Q1[3]
    x2 = Q2[0]
    y2 = Q2[1]
    z2 = Q2[2]
    w2 = Q2[3]
    ### compute the quaternion inner product
    lamb = calculateInnerProduct(Q1, Q2) 
    if lamb < 0:
        w2 = -w2
        x2 = -x2
        y2 = -y2
        z2 = -z2
        lamb = -lamb
    ### calculate interpolation factors (r, s)
    if abs(1-lamb) < epsilon:
        ### the quaternions are nearly parallel, so use linear interpolation
        r = 1 - f
        s = f
    else:
        ### calcuate spherical linear interpolation factors
        alpha = math.acos(lamb)
        gamma = 1.0 / math.sin(alpha)
        r = math.sin((1-f)*alpha) * gamma
        s = math.sin(f*alpha) * gamma

    ### set the interpolated quaternion
    w = r * w1 + s * w2
    x = r * x1 + s * x2
    y = r * y1 + s * y2
    z = r * z1 + s * z2
    Q = [x, y, z, w]
    ### normalize the result
    Q_norm = norm2(Q)
    Q = [x / Q_norm, y / Q_norm, z / Q_norm, w / Q_norm]

    return Q


def convertRobotConfig_dualArm(config, robot, server):
    if server == 0:
        motomanGEO = robot.motomanGEO_p
    else:
        motomanGEO = robot.motomanGEO_e
    ### config: 14*1 list
    configDict = OrderedDict()
    configDict["joint"] = OrderedDict()
    for i in range(len(config)):
        configDict["joint"][robot.motomanRJointNames[i]] = config[i]

    ### then record pose for end effector
    configDict["pose"] = OrderedDict()
    num_joints = p.getNumJoints(motomanGEO, server)
    for i in range(num_joints):
        jointInfo = p.getJointInfo(motomanGEO, i, server)
        if jointInfo[0] == 10 or jointInfo[0] == 20:
            ls = p.getLinkState(motomanGEO, jointInfo[0], physicsClientId=server)
            configDict["pose"][jointInfo[12]] = OrderedDict()
            configDict["pose"][jointInfo[12]]["pos"] = list(ls[0])
            configDict["pose"][jointInfo[12]]["ori"] = list(ls[1])


    return configDict


def convertRobotConfig_singleArm(config, robot, handType, server):
    if server == 0:
        motomanGEO = robot.motomanGEO_p
    else:
        motomanGEO = robot.motomanGEO_e

    if handType == "Left":
        motomanRJointNames = robot.motomanRJointNames[0:7]
    else:
        motomanRJointNames = robot.motomanRJointNames[7:14]
        
    ### config: 7*1 list
    configDict = OrderedDict()
    configDict["joint"] = OrderedDict()
    for i in range(len(config)):
        configDict["joint"][motomanRJointNames[i]] = config[i]

    ### then record pose for end effector
    configDict["pose"] = OrderedDict()
    num_joints = p.getNumJoints(motomanGEO, server)
    for i in range(num_joints):
        jointInfo = p.getJointInfo(motomanGEO, i, server)
        if (handType == "Left" and jointInfo[0] == 10) or (handType == "Right" and jointInfo[0] == 20):
            # print("jointInfo", jointInfo)
            ls = p.getLinkState(motomanGEO, jointInfo[0], physicsClientId=server)
            configDict["pose"][jointInfo[12]] = OrderedDict()
            configDict["pose"][jointInfo[12]]["pos"] = list(ls[0])
            configDict["pose"][jointInfo[12]]["ori"] = list(ls[1])


    return configDict


def generatePreGraspConfig(robot_ee_pose, robot, workspace, planner, handType, server):
    ### this function return pre-grasp poses and their correpsonding arm configs based on the pick poses
    ### Here a pre-grasp pose is a pose which is 10cm away from
    ### the approaching direction (always local z-axis of the gripper) of its corresponding grasp pose

    ### Input: robot_ee_pose (7*1), 
    ###        robot, workspace, planner
    ###        handType: "Left" or "Right", server: planning or executing

    if handType == "Left":
        ee_idx = robot.left_ee_idx
    else:
        ee_idx = robot.right_ee_idx

    temp_quat = robot_ee_pose[3:7]
    ### first change the quaternion to rotation matrix
    temp_rot_matrix = p.getMatrixFromQuaternion(temp_quat)
    ### local z-axis of the end effector
    temp_approaching_direction = [temp_rot_matrix[2], temp_rot_matrix[5], temp_rot_matrix[8]]
    temp_pos = list(np.array(robot_ee_pose[0:3]) - 0.1*np.array(temp_approaching_direction))

    robot_preGraspPose = temp_pos + temp_quat

    ### step 2: get the corresponding IK for the currently examined pose
    q_preGraspIK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO_p,
                            endEffectorLinkIndex=ee_idx,
                            targetPosition=robot_preGraspPose[0:3],
                            targetOrientation=robot_preGraspPose[3:7],
                            lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                            maxNumIterations=20000, residualThreshold=0.0000001,
                            physicsClientId=server)

    if handType == "Left":
        q_preGraspIK = q_preGraspIK[0:7]
    else:
        q_preGraspIK = q_preGraspIK[7:14]

    ### step 3: check the validity of the IK
    isValid = planner.checkIK(
        q_preGraspIK, ee_idx, robot_preGraspPose[0:3], robot, workspace, handType)

    trials = 0
    while (not isValid) and (trials<5):
        ### reset arm configurations
        resetConfiguration = [0.0]*len(robot.armHomeConfiguration)
        robot.resetConfig(resetConfiguration, server)
        q_preGraspIK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO_p,
                                endEffectorLinkIndex=ee_idx,
                                targetPosition=robot_preGraspPose[0:3],
                                targetOrientation=robot_preGraspPose[3:7],
                                lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                                maxNumIterations=20000, residualThreshold=0.0000001,
                                physicsClientId=server)    

        if handType == "Left":
            q_preGraspIK = list(q_preGraspIK[0:7])
        else:
            q_preGraspIK = list(q_preGraspIK[7:14])

        isValid = planner.checkIK(
            q_preGraspIK, ee_idx, robot_preGraspPose[0:3], robot, workspace, handType)

        if isValid: break
        ### otherwise
        trials += 1

    if isValid:
        return robot_preGraspPose, q_preGraspIK
    else:
        return None, None


def generateGraspConfig_objectPose(objectTargetPose, robot, workspace, planner, handType, server):
    ### Input: objectTargetPose {object_name: [[x,y,z],[quaternion]]}
    ###        robot, workspace, planner
    ###        handType: "Left" or "Right"
    ###        server: planning or executing
    ### Output: robot grasp config (7*1), robot pre_grasp config (7*1)

    ### So far the target poses are hard-coded
    ### and Shiyang will integrate his part to provide global target pose


    object_name = objectTargetPose.keys()[0]
    robot_ee_pose = list(objectTargetPose[object_name][0]) + list(objectTargetPose[object_name][0])

    if object_name == "003_cracker_box": 
        robot_ee_pose[2] += 0.03
        robot_ee_pose[3:7] = list(p.getQuaternionFromEuler([0.0, math.pi, 0.0]))

    if object_name == "004_sugar_box":
        robot_ee_pose[2] += 0.02
        robot_ee_pose[3:7] = list(p.getQuaternionFromEuler([0.0, math.pi, 0.0]))

    if handType == "Left":
        ee_idx = robot.left_ee_idx
    else:
        ee_idx = robot.right_ee_idx

    q_graspIK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO_p,
                            endEffectorLinkIndex=ee_idx,
                            targetPosition=robot_ee_pose[0:3],
                            targetOrientation=robot_ee_pose[3:7],
                            lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                            maxNumIterations=20000, residualThreshold=0.0000001,
                            physicsClientId=server)

    if handType == "Left":
        q_graspIK = q_graspIK[0:7]
    else:
        q_graspIK = q_graspIK[7:14]

    isValid = planner.checkIK(
        q_graspIK, ee_idx, robot_ee_pose[0:3], robot, workspace, handType)

    trials = 0
    while (not isValid) and (trials<5):
        ### reset arm configurations
        resetConfiguration = [0.0]*len(robot.armHomeConfiguration)
        robot.resetConfig(resetConfiguration, server)
        q_graspIK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO_p,
                                endEffectorLinkIndex=ee_idx,
                                targetPosition=robot_ee_pose[0:3],
                                targetOrientation=robot_ee_pose[3:7],
                                lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                                maxNumIterations=20000, residualThreshold=0.0000001,
                                physicsClientId=server)

        if handType == "Left":
            q_graspIK = q_graspIK[0:7]
        else:
            q_graspIK = q_graspIK[7:14]

        isValid = planner.checkIK(
            q_graspIK, ee_idx, robot_ee_pose[0:3], robot, workspace, handType)

        if isValid: break
        ### otherwise
        trials += 1

    if isValid:
        ### generate pre-grasp and check its validity
        robot_preGraspPose, q_preGraspIK = generatePreGraspConfig(robot_ee_pose, robot, workspace, planner, handType, server)
        if q_preGraspIK != None:
            return list(q_graspIK), list(q_preGraspIK)
        else:
            return list(q_graspIK), None
    else:
        return None, None


def readObjectTargetPose(scene_index):
    ### Output: pose (list of 7 elements: xyz, quaternion)
    pose_txt = os.getcwd() + "/gripper_poses/" + scene_index + "/objectPose.txt"
    f = open(pose_txt, "r")
    targetPose = {}
    nline = 1
    for line in f:
        line = line.split(" ")
        if nline == 1:
            objectName = line[0]
            targetPose[objectName] = []
        if nline == 2:
            targetPose[objectName].append([float(line[0]), float(line[1]), float(line[2])])
        if nline == 3:
            targetPose[objectName].append([float(line[0]), float(line[1]), float(line[2]), float(line[3])])

        nline += 1

    return targetPose


def loadObjectInScene(targetPose, server):
    obj_path = os.getcwd() + "/mesh/" + targetPose.keys()[0] + "/google_16k/textured.obj"
    _c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=obj_path, meshScale=[1, 1, 1], physicsClientId=server)
    _v = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=obj_path, meshScale=[1, 1, 1], physicsClientId=server)
    pos = targetPose[targetPose.keys()[0]][0]
    quat = targetPose[targetPose.keys()[0]][1]
    _m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v,
                            basePosition=pos, baseOrientation=quat, physicsClientId=server)

    angles = list(p.getEulerFromQuaternion(quat))

    return ObjectMesh(_m, targetPose.keys()[0], pos, quat, angles, obj_path)


def convertPoseToConfig(armPose, robot, workspace, handType, planner, server):
    if handType == "Left":
        ee_idx = robot.left_ee_idx
    else:
        ee_idx = robot.right_ee_idx
    ### get the corresponding IK for the currently examined pose
    q_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO_p,
                            endEffectorLinkIndex=ee_idx,
                            targetPosition=armPose[0:3],
                            targetOrientation=armPose[3:7],
                            lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                            maxNumIterations=20000, residualThreshold=0.0000001,
                            physicsClientId=server)

    if handType == "Left":
        q_IK = q_IK[0:7]
    else:
        q_IK = q_IK[7:14]

    isValid = planner.checkIK(
        q_IK, ee_idx, armPose[0:3], robot, workspace, handType)

    trials = 0
    while (not isValid) and (trials<5):
        ### reset arm configurations
        resetConfiguration = [0.0]*len(robot.armHomeConfiguration)
        robot.resetConfig(resetConfiguration, robot.motomanGEO_p, server)
        q_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO_p,
                                endEffectorLinkIndex=ee_idx,
                                targetPosition=armPose[0:3],
                                targetOrientation=armPose[3:7],
                                lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr,
                                maxNumIterations=20000, residualThreshold=0.0000001,
                                physicsClientId=server)

        if handType == "Left":
            q_IK = q_IK[0:7]
        else:
            q_IK = q_IK[7:14]

        isValid = planner.checkIK(
            q_IK, ee_idx, armPose[0:3], robot, workspace, handType)

        if isValid: break
        ### otherwise
        trials += 1

    if isValid:
        return q_IK
    else:
        return None


def specifyKeyEEposes(workspace, robot, planner, server):
    vacuumPlacementPose = workspace.objectDropCenter + list(p.getQuaternionFromEuler([0.0, math.pi, 0.0]))
    vacuumPlacementConfig = convertPoseToConfig(vacuumPlacementPose, robot, workspace, "Left", planner, server)

    ## specify the pose for handoff the object for the left arm
    # vacuumHandoffPose = workspace.objectHandOffCenter + list(p.getQuaternionFromEuler([0.0, math.pi, 0.0]))
    # vacuumHandoffConfig = convertPoseToConfig(vacuumHandoffPose, robot, workspace, "Left", planner, server)

    ### specify the pose after finish the task for the left arm
    # vacuumFinishPose = workspace.leftArmFinishCenter + list(p.getQuaternionFromEuler([math.pi/2, math.pi/2, 0.0]))
    # vacuumFinishConfig = convertPoseToConfig(vacuumFinishPose, "Left")

    ### specify the pose for placing the object for the right arm
    # fingerPlacementPose = \
    #     self.workspace.objectDropCenter + list(p.getQuaternionFromEuler([0.0, math.pi, 0.0]))
    # fingerPlacementConfig = convertPoseToConfig(fingerPlacementPose, "Right")

    return list(vacuumPlacementConfig)


######################## The following code snippets are not used but kept for legacy ########################
##############################################################################################################

def genServers(exp_mode):
    ### input -> exp_node: three modes
    ### output -> [planningServer, executingServer]
    if exp_mode == "e":
        ### experiment mode: use it when performing large-scale experiment
        ### in this case, disable all GUI to speed up computational time
        planningServer = p.connect(p.DIRECT)
        executingServer = p.connect(p.DIRECT)
    elif exp_mode == "v":
        ### visualization mode: use this when you only want to see
        ### the visualization of the true scene
        planningServer = p.connect(p.DIRECT)
        executingServer = p.connect(p.GUI)
        # planningServer = bc.BulletClient(connection_mode=p.DIRECT)
        # executingServer = bc.BulletClient(connection_mode=p.GUI)

    elif exp_mode == "p":
        ### planning mode: use this when you want to see how planning performs
        ### the physics are disabled in this mode
        planningServer = p.connect(p.GUI)
        executingServer = p.connect(p.DIRECT)

    print("planningServer: " + str(planningServer))
    print("executingServer: " + str(executingServer))

    return [planningServer, executingServer]



class ObjectMesh:
    def __init__(self, m, objName, pos, quat, angles, meshFile):
        self.m = m
        self.objName = objName
        self.pos = pos
        self.quat = quat
        self.angles = angles
        self.meshFile = meshFile

    def update_m(self, m):
        self.m = m

    def update_pos(self, pos):
        self.pos = pos

    def update_quat(self, quat):
        self.quat = quat

    def update_angles(self, angles):
        self.angles = angles


def dropObjectOnTable1(mesh_folder, obj_name, obj_configs_angles, tablePosition, table_dim, dropHeight, serverClientID):
    ### This function drop an obj of a specified configs and random position (table region)
    ### on the table
    ### input -> mesh_folder, the directory where the meshes are stored
    ###          obj_name, the name of the object you want to drop
    ###          obj_configs_angles: a list of tuple(alpha, beta, gamma)
    ###          tablePosition: [x, y, z(height)]
    ###          table_dim: np.array([x, y, z(height)])
    ###          dropHeight: float value, indicating at what height will the object be dropped
    ###          serverClientID: specified which server to use, Plan or Execute?
    ### output -> object mesh produced which is on the table

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
    obj_path = os.path.join(mesh_folder, obj_name, "google_16k/textured.obj")
    _c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=obj_path, meshScale=[1, 1, 1], physicsClientId=serverClientID)
    _v = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=obj_path, meshScale=[1, 1, 1], physicsClientId=serverClientID)
    ### random position given the table position and table_dim
    temp_pos = [random.uniform(tablePosition[0]-table_dim[0]/2+0.1, tablePosition[0]+table_dim[0]/2-0.1), \
                random.uniform(tablePosition[1]+0.1, tablePosition[1]+table_dim[1]/2-0.1), \
            tablePosition[2]+table_dim[2]/2+dropHeight
            ]
    ### select one configuration
    temp_angles = random.choice(obj_configs_angles)
    ### add some randomness on the orientation around z-axis
    temp_angles[2] = temp_angles[2] + random.uniform(-180, 180)
    temp_quat = p.getQuaternionFromEuler([i*math.pi/180 for i in temp_angles])
    ### create the mesh for the object
    # _m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v,
    #                         basePosition=pos, baseOrientation=quat, physicsClientId=serverClientID)
    _m = p.createMultiBody(baseMass=massList[obj_name], baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v,
                            basePosition=temp_pos, baseOrientation=temp_quat, physicsClientId=serverClientID)

    ### ready to drop the object
    # raw_input("press ENTER to drop the object")
    p.setGravity(0.0, 0.0, -9.8, physicsClientId=serverClientID)
    p.setRealTimeSimulation(enableRealTimeSimulation=1, physicsClientId=serverClientID)

    pos, quat = p.getBasePositionAndOrientation(_m, physicsClientId=serverClientID)
    object_pose = ObjectMesh(_m, obj_name, list(pos), list(quat), list(p.getEulerFromQuaternion(list(quat))), obj_path)
    print("object position on the table: " + str(object_pose.pos))
    print("object orientation on the table: " + str(object_pose.angles))

    return object_pose