#!/usr/bin/env python

from __future__ import division
import pybullet as p
import pybullet_data

import sys
import os
import time
import IPython
import numpy as np
import math

import utils
from MotomanRobot import MotomanRobot
from Workspace import Workspace

if __name__ == '__main__':

    planningServer = p.connect(p.DIRECT)
    executingServer = p.connect(p.GUI)
    servers = [planningServer, executingServer]

    ### introduce the robot
    robot = MotomanRobot(servers)
    ### generate the scene (table or shelf, or whatever)
    workspace = Workspace("Table", robot.BasePosition, servers)
    ### get all the geometries from the robot and the workspace
    planningGeometries = robot.known_geometries_planning + workspace.known_geometries_planning
    executingGeometries = robot.known_geometries_executing + workspace.known_geometries_executing

    loc = "/home/rui/Documents/research/motoman_ws/src/pybullet_motoman/src"


    ### now let's plot each node as a circle in the scene
    f_reachable = open(loc+"/reachable.txt", "r")
    for line in f_reachable:
        line = line.split(" ")
        xyz = [float(line[0]), float(line[1]), float(line[2])]
        spot_v = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[0,1,0,1], physicsClientId=executingServer)
        spot_m = p.createMultiBody(baseVisualShapeIndex=spot_v, basePosition=xyz, physicsClientId=executingServer)

    ### now let's plot each node as a circle in the scene
    f_collision = open(loc+"/collision.txt", "r")
    for line in f_collision:
        line = line.split(" ")
        xyz = [float(line[0]), float(line[1]), float(line[2])]
        spot_v = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[1,0,0,1], physicsClientId=executingServer)
        spot_m = p.createMultiBody(baseVisualShapeIndex=spot_v, basePosition=xyz, physicsClientId=executingServer)

    ### now let's plot each node as a circle in the scene
    f_IKfail = open(loc+"/IKfail.txt", "r")
    for line in f_IKfail:
        line = line.split(" ")
        xyz = [float(line[0]), float(line[1]), float(line[2])]
        spot_v = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.005, rgbaColor=[0,0,1,1], physicsClientId=executingServer)
        spot_m = p.createMultiBody(baseVisualShapeIndex=spot_v, basePosition=xyz, physicsClientId=executingServer)


    time.sleep(10000)