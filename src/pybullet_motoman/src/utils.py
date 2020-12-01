### utils file contains some of the common functionalities
### shared by lots of experimental setting

from __future__ import division
import pybullet as p
import pybullet_data

import os
import random
import math

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
    elif exp_mode == "p":
        ### planning mode: use this when you want to see how planning performs
        ### the physics are disabled in this mode
        planningServer = p.connect(p.GUI)
        executingServer = p.connect(p.DIRECT)

    return [planningServer, executingServer]


def dropObjectOnTable(obj_name, obj_configs_angles, tablePosition, table_dim, dropHeight, serverClientID):
    ### This function drop an obj of a specified configs and random position (table region)
    ### on the table
    ### input -> obj_name, the name of the object you want to drop
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
    loc = "/home/rui/Documents/research/motoman_ws/src/pybullet_motoman/src/"
    _c = p.createCollisionShape(shapeType=p.GEOM_MESH,
                    fileName=loc+"/mesh/"+obj_name+"/google_16k/textured.obj", meshScale=[1, 1, 1], physicsClientId=serverClientID)
    _v = p.createVisualShape(shapeType=p.GEOM_MESH,
                    fileName=loc+"/mesh/"+obj_name+"/google_16k/textured.obj", meshScale=[1, 1, 1], physicsClientId=serverClientID)
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
    object_pose = ObjectMesh(_m, obj_name, list(pos), list(quat), list(p.getEulerFromQuaternion(list(quat))), "/mesh/"+obj_name+"/google_16k/textured.obj")
    print("object position on the table: " + str(object_pose.pos))
    print("object orientation on the table: " + str(object_pose.angles))

    return object_pose


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




    