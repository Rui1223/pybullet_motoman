#!/usr/bin/env python
from __future__ import division

import pybullet as p
import pybullet_data

from collections import OrderedDict
import os
import random
import math
import numpy as np
import time
import IPython

### This file defines workspace for the table ###

class WorkspaceTable(object):
    def __init__(self,
        robotBasePosition,
        standingBase_dim, table_dim, table_offset_x, transitCenterHeight,
        mesh_path, isPhysicsTurnOn, server):
        ### get the server
        self.server = server
        self.mesh_path = mesh_path
        self.known_geometries = []
        self.object_geometries = OrderedDict()
        self.createTableScene(
            robotBasePosition, standingBase_dim, table_dim, table_offset_x, isPhysicsTurnOn)
        ### specify the transit center
        self.objectTransitCenter = [
            self.tablePosition[0], self.tablePosition[1], self.tablePosition[2]+transitCenterHeight]



    def createTableScene(self, 
        robotBasePosition, standingBase_dim, table_dim, table_offset_x, isPhysicsTurnOn):
        print("---------Enter to table scene!----------")

        ################ create the known geometries - standingBase  ####################
        self.standingBase_dim = np.array(standingBase_dim)
        self.standingBasePosition = [
            robotBasePosition[0], robotBasePosition[1], robotBasePosition[2]-self.standingBase_dim[2]/2-0.005]
        self.standingBase_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                    halfExtents=self.standingBase_dim/2, physicsClientId=self.server)
        self.standingBase_v = p.createVisualShape(shapeType=p.GEOM_BOX,
                                    halfExtents=self.standingBase_dim/2, physicsClientId=self.server)
        if isPhysicsTurnOn == True:
            self.standingBaseM = p.createMultiBody(
                baseCollisionShapeIndex=self.standingBase_c, baseVisualShapeIndex=self.standingBase_v,
                basePosition=self.standingBasePosition, physicsClientId=self.server)
        else:
            self.standingBaseM = p.createMultiBody(
                baseCollisionShapeIndex=self.standingBase_c, baseVisualShapeIndex=self.standingBase_v,
                basePosition=self.standingBasePosition, physicsClientId=self.server)            
        print("standing base: " + str(self.standingBaseM))
        self.known_geometries.append(self.standingBaseM)
        #################################################################################


        ################ create the known geometries - table  ###########################
        # self.table_dim = np.array([table_dim[0], table_dim[1], table_dim[2]+self.standingBase_dim[2]+0.005])
        self.table_dim = np.array([table_dim[0], table_dim[1], table_dim[2]])
        self.tablePosition = [
            table_offset_x+self.table_dim[0]/2, robotBasePosition[1], robotBasePosition[2]+(self.table_dim[2]/2-self.standingBase_dim[2]-0.005)]

        self.table_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                halfExtents=self.table_dim/2, physicsClientId=self.server)
        self.table_v = p.createVisualShape(shapeType=p.GEOM_BOX,
                                halfExtents=self.table_dim/2, physicsClientId=self.server)
        self.tableM = p.createMultiBody(baseCollisionShapeIndex=self.table_c, baseVisualShapeIndex=self.table_v,
                                            basePosition=self.tablePosition, physicsClientId=self.server)
        print("table: " + str(self.tableM))
        self.known_geometries.append(self.tableM)
        #################################################################################


    def dropObjectOnTable(self, obj_name, dropHeight):
        ### This function drop an obj of a specified configs and random position (table region)
        ### on the table
        ### input -> mesh_folder, the directory where the meshes are stored
        ###          obj_name, the name of the object you want to drop
        ###          tablePosition: [x, y, z(height)]
        ###          table_dim: np.array([x, y, z(height)])
        ###          dropHeight: float value, indicating at what height will the object be dropped
        ###          server: specified which server to use
        ### Output -> object mesh produced which is on the table
        ###           (but you should no access to the ground truth pose so as to mimic the reality)


        object_configs_angles = {
            "003_cracker_box": [[-0.035, -0.336, 87.775], [89.801, -2.119, 112.705], [-25.498, -84.700, 110.177]],
            "004_sugar_box": [[-0.166, -0.100, -144.075], [90.822, -1.909, 67.882], [-7.177, -79.030, 102.698]],
            "006_mustard_bottle": [[0.006, 0.061, -135.114], [87.134, -1.560, 89.805]],
            "008_pudding_box": [[89.426, 0.412, -96.268], [-0.721, 0.300, -138.733]],
            "010_potted_meat_can": [[-0.131, -0.061, 97.479], [87.863, -1.266, -65.330]],
            "021_bleach_cleanser": [[-0.103, -0.082, -39.439], [-84.349, -1.891, -177.925]]
        }

        massList = {
            "003_cracker_box": 2.32,
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

        obj_path = os.path.join(self.mesh_path, obj_name, "google_16k/textured.obj")
        _c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=obj_path, meshScale=[1, 1, 1], physicsClientId=self.server)
        _v = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=obj_path, meshScale=[1, 1, 1], physicsClientId=self.server)
        ### random position given the table position and table_dim
        temp_pos = [random.uniform(self.tablePosition[0]-self.table_dim[0]/2+0.1, self.tablePosition[0]+self.table_dim[0]/2-0.1), \
                    random.uniform(self.tablePosition[1]+0.1, self.tablePosition[1]+self.table_dim[1]/2-0.1), \
                    self.tablePosition[2]+self.table_dim[2]/2+dropHeight]
        print("temp_pos")
        print(temp_pos)

        ### select one configuration
        temp_angles = random.choice(object_configs_angles[obj_name])
        ### add some randomness on the orientation around z-axis
        temp_angles[2] = temp_angles[2] + random.uniform(-180, 180)
        temp_quat = p.getQuaternionFromEuler([i*math.pi/180 for i in temp_angles])
        ### create the mesh for the object
        # _m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v,
        #                         basePosition=pos, baseOrientation=quat, physicsClientId=server)
        _m = p.createMultiBody(baseMass=massList[obj_name], baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v,
                                basePosition=temp_pos, baseOrientation=temp_quat, physicsClientId=self.server)

        ### ready to drop the object
        p.setGravity(0.0, 0.0, -9.8, physicsClientId=self.server)
        p.setRealTimeSimulation(enableRealTimeSimulation=1, physicsClientId=self.server)
        ### wait for one second after the drop
        time.sleep(1.5)
        p.setRealTimeSimulation(enableRealTimeSimulation=0, physicsClientId=self.server)

        ### register this object
        self.obj_name = obj_name
        self.object_geometries[_m] = self.obj_name


    def fixAnObjectOnTable(self, obj_name):

        ### This is just a test function

        object_configs_angles = {
            "003_cracker_box": [[-0.035, -0.336, 87.775], [89.801, -2.119, 112.705], [-25.498, -84.700, 110.177]],
            "004_sugar_box": [[-0.166, -0.100, -144.075], [90.822, -1.909, 67.882], [-7.177, -79.030, 102.698]],
            "006_mustard_bottle": [[0.006, 0.061, -135.114], [87.134, -1.560, 89.805]],
            "008_pudding_box": [[89.426, 0.412, -96.268], [-0.721, 0.300, -138.733]],
            "010_potted_meat_can": [[-0.131, -0.061, 97.479], [87.863, -1.266, -65.330]],
            "021_bleach_cleanser": [[-0.103, -0.082, -39.439], [-84.349, -1.891, -177.925]]
        }

        massList = {
            "003_cracker_box": 2.32,
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

        obj_path = os.path.join(self.mesh_path, obj_name, "google_16k/textured.obj")
        _c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=obj_path, meshScale=[1, 1, 1], physicsClientId=self.server)
        _v = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=obj_path, meshScale=[1, 1, 1], physicsClientId=self.server)
        ### random position given the table position and table_dim
        # temp_pos = [random.uniform(self.tablePosition[0]-self.table_dim[0]/2+0.1, self.tablePosition[0]+self.table_dim[0]/2-0.1), \
        #             random.uniform(self.tablePosition[1]+0.1, self.tablePosition[1]+self.table_dim[1]/2-0.1), \
        #             self.tablePosition[2]+self.table_dim[2]/2]
        temp_pos = [0.80, 0.45, 0.61 + 0.025]
        temp_quat = [0.0, 1.0, 0.0, 1.0]


        ### select one configuration
        # temp_angles = object_configs_angles[obj_name][0]
        ### add some randomness on the orientation around z-axis
        # temp_angles[2] = temp_angles[2] + random.uniform(-180, 180)
        # temp_quat = p.getQuaternionFromEuler([i*math.pi/180 for i in temp_angles])
        ### create the mesh for the object
        # _m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v,
        #                         basePosition=pos, baseOrientation=quat, physicsClientId=server)
        _m = p.createMultiBody(baseMass=massList[obj_name], baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v,
                                basePosition=temp_pos, baseOrientation=temp_quat, physicsClientId=self.server)

        ### ready to drop the object
        # p.setGravity(0.0, 0.0, -9.8, physicsClientId=self.server)
        # p.setRealTimeSimulation(enableRealTimeSimulation=1, physicsClientId=self.server)
        ### wait for one second after the drop
        time.sleep(1.5)
        # p.setRealTimeSimulation(enableRealTimeSimulation=0, physicsClientId=self.server)

        ### register this object
        self.obj_name = obj_name
        self.object_geometries[_m] = self.obj_name

    ### This function is disabled
    def getObjectInfo(self):
        ### this function is called to get the object information
        ### which includes (1) the object name
        ### (2) current object pose
        obj_pose = p.getBasePositionAndOrientation(
            self.object_geometries.keys()[0], physicsClientId=self.server)
        obj_pose = [list(obj_pose[0]), list(obj_pose[1])]
        return self.obj_name, obj_pose


    def updateObjectMesh(self, object_pose):
        ### this function is called to update the object pose
        ### NOTE: it should be only called by planning scene
        ### here the object_pose is a msg of ObjectPoseBox (dims, position, orientation)

        ### first check if the object is already in the scene
        if not self.object_geometries:
            ### no object is registered, so we need to add the object
            _c = p.createCollisionShape(
                shapeType=p.GEOM_BOX, halfExtents=np.array(object_pose.dims)/2, 
                            meshScale=[1, 1, 1], physicsClientId=self.server)
            _v = p.createVisualShape(
                shapeType=p.GEOM_BOX, halfExtents=np.array(object_pose.dims)/2, 
                        meshScale=[1, 1, 1], rgbaColor=[0.35, 0.35, 0.35, 1], physicsClientId=self.server)
            _m = p.createMultiBody(
                baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v,
                basePosition=object_pose.position, baseOrientation=object_pose.orientation, 
                physicsClientId=self.server)
            self.object_geometries[_m] = [[list(object_pose.position), list(object_pose.orientation)], object_pose.dims]
            # print(self.object_geometries)

        else:
            ### we first need to remove the current object mesh
            # print(self.object_geometries.keys()[0])
            p.removeBody(self.object_geometries.keys()[0], physicsClientId=self.server)
            self.object_geometries = OrderedDict()
            ### generate the new object mesh
            _c = p.createCollisionShape(
                shapeType=p.GEOM_BOX, halfExtents=np.array(object_pose.dims)/2, 
                            meshScale=[1, 1, 1], physicsClientId=self.server)
            _v = p.createVisualShape(
                shapeType=p.GEOM_BOX, halfExtents=np.array(object_pose.dims)/2, 
                        meshScale=[1, 1, 1], rgbaColor=[0.35, 0.35, 0.35, 1], physicsClientId=self.server)
            _m = p.createMultiBody(
                baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v,
                basePosition=object_pose.position, baseOrientation=object_pose.orientation, 
                physicsClientId=self.server)
            self.object_geometries[_m] = [[list(object_pose.position), list(object_pose.orientation)], object_pose.dims]
            # print(self.object_geometries)


    def updateObjectGeomeotry_BoundingBox(self, object_pose, object_dim):
        ### This function update the object given
        ### (1) object_pose Pose3D (position(x,y,z), orientation(x,y,z,w))
        ### (2) object_dim BoundingBox3D (x, y, z)
        ### we assume the object is modelled as the bounding box in the planning scene
        object_dim = np.array([object_dim[0], object_dim[1], object_dim[2]])
        object_pose = [[object_pose.position.x, object_pose.position.y, object_pose.position.z], \
            [object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w]]

        if not bool(self.object_geometries):
            ### no object geometries has been introduced before
            ### then create the object geometry
            geo_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                halfExtents=object_dim/2, physicsClientId=self.server)
            geo_v = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=object_dim/2,
                        rgbaColor=[128.0/255.0, 128.0/255.0, 128.0/255.0, 0.8], physicsClientId=self.server)
            tableM = p.createMultiBody(baseCollisionShapeIndex=geo_c, baseVisualShapeIndex=geo_v,
                    basePosition=object_pose[0], baseOrientation=object_pose[1], physicsClientId=self.server)
        else:
            print("The idea is to update the mesh")
            print("will come back later")


    def enablePhysicsEnv(self):
        p.setGravity(0.0, 0.0, -9.8, physicsClientId=self.server)
        p.setRealTimeSimulation(enableRealTimeSimulation=1, physicsClientId=self.server)


    def disablePhysicsEnv(self):
        p.setRealTimeSimulation(enableRealTimeSimulation=0, physicsClientId=self.server)        