from __future__ import division
import pybullet as p
import pybullet_data

import numpy as np


class Workspace(object):
    ### This specified a workspace where the robot is performing the task
    def __init__(self, theme, robotBasePosition, servers):
        ### specify the theme of the workspace
        self.theme = theme
        ### get the server
        self.planningServer = servers[0]
        self.executingServer = servers[1]
        ### collect geometries from the workspace
        self.known_geometries_planning = []
        self.known_geometries_executing = []        

        if self.theme == "Table": 
            self.createTableScene(robotBasePosition)
            self.objectDropCenter = [
                self.tablePosition[0], self.tablePosition[1], self.tablePosition[2]+0.8]
            self.objectHandOffCenter = [
                self.tablePosition[0], self.tablePosition[1]-0.01, self.tablePosition[2]+0.9]
            self.leftArmFinishCenter = [
                self.tablePosition[0]-0.3, self.tablePosition[1]+0.3, self.tablePosition[2]+1.0]


    def createTableScene(self, robotBasePosition):
        print "---------Enter to table scene!----------"

        ################ create the known geometries - standingBase  ####################
        self.standingBase_dim = np.array([0.915, 0.62, 0.19])
        self.standingBasePosition = [robotBasePosition[0], robotBasePosition[1], robotBasePosition[2]-self.standingBase_dim[2]/2-0.005]
        self.standingBase_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
                                    halfExtents=self.standingBase_dim/2, physicsClientId=self.planningServer)
        self.standingBase_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, 
                                    halfExtents=self.standingBase_dim/2, physicsClientId=self.planningServer)
        self.standingBaseM_p = p.createMultiBody(baseCollisionShapeIndex=self.standingBase_c_p, baseVisualShapeIndex=self.standingBase_v_p,
                                    basePosition=self.standingBasePosition, physicsClientId=self.planningServer)
        self.standingBase_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
                                    halfExtents=self.standingBase_dim/2, physicsClientId=self.executingServer)
        self.standingBase_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, 
                                    halfExtents=self.standingBase_dim/2, physicsClientId=self.executingServer)
        self.standingBaseM_e = p.createMultiBody(baseCollisionShapeIndex=self.standingBase_c_e, baseVisualShapeIndex=self.standingBase_v_e,
                                    basePosition=self.standingBasePosition, physicsClientId=self.executingServer)
        print "standing base: " + str(self.standingBaseM_e)
        self.known_geometries_planning.append(self.standingBaseM_p)
        self.known_geometries_executing.append(self.standingBaseM_e)
        #################################################################################


        ################ create the known geometries - table  ###########################
        # self.table_dim = np.array([0.58, 1.44, 0.58+self.standingBase_dim[2]])
        self.table_dim = np.array([0.58, 1.32, 0.58+self.standingBase_dim[2]+0.005])
        # self.tablePosition = [0.51+self.table_dim[0]/2, 0.78-self.table_dim[1]/2, robotBasePosition[2]+(self.table_dim[2]/2-self.standingBase_dim[2]-0.005)]
        self.tablePosition = [0.51+self.table_dim[0]/2, robotBasePosition[1], robotBasePosition[2]+(self.table_dim[2]/2-self.standingBase_dim[2]-0.005)]
        self.table_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                halfExtents=self.table_dim/2, physicsClientId=self.planningServer)
        self.table_v_p = p.createVisualShape(shapeType=p.GEOM_BOX,
                                halfExtents=self.table_dim/2, physicsClientId=self.planningServer)
        self.tableM_p = p.createMultiBody(baseCollisionShapeIndex=self.table_c_p, baseVisualShapeIndex=self.table_v_p,
                                            basePosition=self.tablePosition, physicsClientId=self.planningServer)
        self.table_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                halfExtents=self.table_dim/2, physicsClientId=self.executingServer)
        self.table_v_e = p.createVisualShape(shapeType=p.GEOM_BOX,
                                halfExtents=self.table_dim/2, physicsClientId=self.executingServer)
        self.tableM_e = p.createMultiBody(baseCollisionShapeIndex=self.table_c_e, baseVisualShapeIndex=self.table_v_e,
                                            basePosition=self.tablePosition, physicsClientId=self.executingServer)
        print "table: " + str(self.tableM_e)
        self.known_geometries_planning.append(self.tableM_p)
        self.known_geometries_executing.append(self.tableM_e)
        #################################################################################