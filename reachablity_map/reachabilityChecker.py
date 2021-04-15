#!/usr/bin/env python

from __future__ import division
import pybullet as p
import pybullet_data

import sys
import os
import time
from collections import OrderedDict
import IPython
import numpy as np
import math

import utils
from MotomanRobot import MotomanRobot
from Workspace import Workspace
from Camera import AzureKineticCamera

if __name__ == '__main__':

    planningServer = p.connect(p.DIRECT)
    executingServer = p.connect(p.DIRECT)
    servers = [planningServer, executingServer]

    ### introduce the robot
    robot = MotomanRobot(servers)
    ### generate the scene (table or shelf, or whatever)
    workspace = Workspace("Table", robot.BasePosition, servers)
    ### get all the geometries from the robot and the workspace
    planningGeometries = robot.known_geometries_planning + workspace.known_geometries_planning
    executingGeometries = robot.known_geometries_executing + workspace.known_geometries_executing

    x_range = [workspace.tablePosition[0]-workspace.table_dim[0]/2, workspace.tablePosition[0]+workspace.table_dim[0]/2]
    y_range = [workspace.tablePosition[1]-workspace.table_dim[1]/2, workspace.tablePosition[1]+workspace.table_dim[1]/2]
    z_range = [workspace.tablePosition[2]+workspace.table_dim[2]/2+0.01, workspace.tablePosition[2]+workspace.table_dim[2]/2+0.01+0.3]
    x_interval = 0.02
    y_interval = 0.02
    z_interval = 0.02
    error_threshold = 0.015

    # goalEuler = [0.0, math.pi, 0.0]
    # temp_goal_pose_quat = p.getQuaternionFromEuler([goalEuler[0], goalEuler[1], 
    #                                             goalEuler[2]])
    # goal_pose_quat = [temp_goal_pose_quat[0], temp_goal_pose_quat[1], temp_goal_pose_quat[3], temp_goal_pose_quat[2]]
    # print("goal_pose_quat: ", goal_pose_quat)

    loc = "/home/rui/Documents/research/motoman_ws/src/pybullet_motoman/src"
    f_reachable = open(loc+"/reachable.txt", "w")
    f_IKfail = open(loc+"/IKfail.txt", "w")
    f_collision = open(loc+"/collision.txt", "w")

    ### left hand reachablilty
    for x in np.arange(x_range[0], x_range[1], x_interval):
        for y in np.arange(0.0, y_range[1], y_interval):
            for z in np.arange(z_range[0], z_range[1], z_interval):
                # print("\n")
                desired_pos = [x, y, z]
                print(desired_pos)
                ### get the IK
                q_graspIK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO_p,
                                        endEffectorLinkIndex=robot.left_ee_idx,
                                        targetPosition=desired_pos,
                                        targetOrientation=[1.0, 0.0, 0.0, 0.0],
                                        lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr, restPoses=robot.rp,
                                        maxNumIterations=20000, residualThreshold=0.0000001,
                                        physicsClientId=planningServer)
                for j in range(1, 8):
                    result_p = p.resetJointState(robot.motomanGEO_p, j, q_graspIK[j-1], physicsClientId=planningServer)
                p.stepSimulation(physicsClientId=planningServer)
                ls = p.getLinkState(
                    bodyUniqueId=robot.motomanGEO_p, linkIndex=robot.left_ee_idx, computeForwardKinematics=True, physicsClientId=planningServer)
                ### check first if it is IK reachable
                error = utils.calIKerror(desired_pos, ls[0])
                if (error >= error_threshold):
                    # print("this pose is not IK reachable!")
                    f_IKfail.write(str(desired_pos[0]) + " " + str(desired_pos[1]) + " " + str(desired_pos[2]) + "\n")
                    continue
                ### if it is IK reachable, then check collision (safety always on top of priority)
                isCollisionSelf = utils.collisionCheck_selfCollision(robot.motomanGEO_p, planningServer)
                # if isCollisionSelf:
                #     print("self collision")
                isCollisionKnownObs = utils.collisionCheck_knownObs(
                                robot.motomanGEO_p, workspace.known_geometries_planning, planningServer)
                # if isCollisionKnownObs:
                #     print("collide with known obstacles")
                if isCollisionSelf or isCollisionKnownObs:
                    # print("Collision with either robot itself or known obstacles!")
                    f_collision.write(str(desired_pos[0]) + " " + str(desired_pos[1]) + " " + str(desired_pos[2]) + "\n")
                    continue

                ### if reach here then the pose is reachable and no collision with known obstacles
                # print("valid pose")
                f_reachable.write(str(desired_pos[0]) + " " + str(desired_pos[1]) + " " + str(desired_pos[2]) + "\n")


    print("switch to right hand")

    ### right hand reachability
    for x in np.arange(x_range[0], x_range[1], x_interval):
        for y in np.arange(y_range[0], 0.0, y_interval):
            for z in np.arange(z_range[0], z_range[1], z_interval):
                # print("\n")
                desired_pos = [x, y, z]
                print(desired_pos)
                ### get the IK
                q_graspIK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO_p,
                                        endEffectorLinkIndex=robot.right_ee_idx,
                                        targetPosition=desired_pos,
                                        targetOrientation=[1.0, 0.0, 0.0, 0.0],
                                        lowerLimits=robot.ll, upperLimits=robot.ul, jointRanges=robot.jr, restPoses=robot.rp,
                                        maxNumIterations=20000, residualThreshold=0.0000001,
                                        physicsClientId=planningServer)
                for j in range(11, 18):
                    result_p = p.resetJointState(robot.motomanGEO_p, j, q_graspIK[j-4], physicsClientId=planningServer)
                p.stepSimulation(physicsClientId=planningServer)
                ls = p.getLinkState(
                    bodyUniqueId=robot.motomanGEO_p, linkIndex=robot.right_ee_idx, computeForwardKinematics=True, physicsClientId=planningServer)
                ### check first if it is IK reachable
                error = utils.calIKerror(desired_pos, ls[0])
                if (error >= error_threshold):
                    # print("this pose is not IK reachable!")
                    f_IKfail.write(str(desired_pos[0]) + " " + str(desired_pos[1]) + " " + str(desired_pos[2]) + "\n")
                    continue
                ### if it is IK reachable, then check collision (safety always on top of priority)
                isCollisionSelf = utils.collisionCheck_selfCollision(robot.motomanGEO_p, planningServer)
                # if isCollisionSelf:
                #     print("self collision")
                isCollisionKnownObs = utils.collisionCheck_knownObs(
                                robot.motomanGEO_p, workspace.known_geometries_planning, planningServer)
                # if isCollisionKnownObs:
                #     print("collide with known obstacles")

                if isCollisionSelf or isCollisionKnownObs:
                    # print("Collision with either robot itself or known obstacles!")
                    f_collision.write(str(desired_pos[0]) + " " + str(desired_pos[1]) + " " + str(desired_pos[2]) + "\n")
                    continue

                ### if reach here then the pose is reachable and no collision with known obstacles
                # print("valid pose")
                f_reachable.write(str(desired_pos[0]) + " " + str(desired_pos[1]) + " " + str(desired_pos[2]) + "\n")


    f_IKfail.close()
    f_collision.close()
    f_reachable.close()

    time.sleep(3)

