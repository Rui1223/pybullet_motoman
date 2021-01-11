#!/usr/bin/env python
from __future__ import division

import pybullet as p
import pybullet_data

import numpy as np

### This file defines workspace for the table ###

class WorkspaceTable(object):
    def __init__(self, 
        robotBasePosition,
        standingBase_dim, table_dim, table_offset_x, transitCenterHeight,
        server):
        ### get the server
        self.server = server
        self.known_geometries = []
        self.createTableScene(robotBasePosition, standingBase_dim, table_dim, table_offset_x)
        ### specify the transit center
        self.objectTransitCenter = [
            self.tablePosition[0], self.tablePosition[1], self.tablePosition[2]+transitCenterHeight]


    def createTableScene(self, robotBasePosition, standingBase_dim, table_dim, table_offset_x):
        print("---------Enter to table scene!----------")

        ################ create the known geometries - standingBase  ####################
        self.standingBase_dim = np.array(standingBase_dim)
        self.standingBasePosition = [
            robotBasePosition[0], robotBasePosition[1], robotBasePosition[2]-self.standingBase_dim[2]/2-0.005]
        self.standingBase_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                    halfExtents=self.standingBase_dim/2, physicsClientId=self.server)
        self.standingBase_v = p.createVisualShape(shapeType=p.GEOM_BOX,
                                    halfExtents=self.standingBase_dim/2, physicsClientId=self.server)
        self.standingBaseM = p.createMultiBody(baseCollisionShapeIndex=self.standingBase_c, baseVisualShapeIndex=self.standingBase_v,
                                    basePosition=self.standingBasePosition, physicsClientId=self.server)
        print("standing base: " + str(self.standingBaseM))
        self.known_geometries.append(self.standingBaseM)
        #################################################################################


        ################ create the known geometries - table  ###########################
        self.table_dim = np.array([table_dim[0], table_dim[1], table_dim[2]+self.standingBase_dim[2]+0.005])
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