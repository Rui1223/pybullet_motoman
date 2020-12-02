#!/usr/bin/env python

from __future__ import division
import pybullet as p
import pybullet_data

import sys
import os
import time
from collections import OrderedDict
import IPython

import cv2
from cv_bridge import CvBridge, CvBridgeError

import utils
from MotomanRobot import MotomanRobot
from Workspace import Workspace
from Camera import AzureKineticCamera

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image


if __name__ == '__main__':

	exp_mode = "v"
	object_name = sys.argv[1] ### select an object to be placed in the scene
	scene_index = sys.argv[2] ### the id of the current scene/experiment
	saveImages = (sys.argv[3] in ('y', 'Y')) ### decide whether to save images or not

	servers = utils.genServers(exp_mode)

	### introduce the robot
	robot = MotomanRobot(servers)
	### generate the scene (table or shelf, or whatever)
	workspace = Workspace("Table", robot.BasePosition, servers)
	### get all the geometries from the robot and the workspace
	planningGeometries = robot.known_geometries_planning + workspace.known_geometries_planning
	executingGeometries = robot.known_geometries_executing + workspace.known_geometries_executing
	### set the camera ready
	camera = AzureKineticCamera(workspace.tablePosition, workspace.table_dim, scene_index, saveImages)

	object_configs_angles = {
		"003_cracker_box": [[-0.035, -0.336, 87.775], [89.801, -2.119, 112.705], [-25.498, -84.700, 110.177]],
		"004_sugar_box": [[-0.166, -0.100, -144.075], [90.822, -1.909, 67.882], [-7.177, -79.030, 102.698]],
		"006_mustard_bottle": [[0.006, 0.061, -135.114], [87.134, -1.560, 89.805]],
		"008_pudding_box": [[89.426, 0.412, -96.268], [-0.721, 0.300, -138.733]],
		"010_potted_meat_can": [[-0.131, -0.061, 97.479], [87.863, -1.266, -65.330]],
		"021_bleach_cleanser": [[-0.103, -0.082, -39.439], [-84.349, -1.891, -177.925]] 
	}

	### add an object
	object_pose = utils.dropObjectOnTable(
		object_name, object_configs_angles[object_name], workspace.tablePosition, workspace.table_dim, 0.15, servers[1])
	time.sleep(3)

	### when the scene is ready, let's publish some message to the scene
	rgbImg_pub = rospy.Publisher('rgbImages', Image, queue_size=10)
	rospy.init_node('image_taker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		### get the time stamp
		time_stamp = rospy.get_time()
		hello_str = "take image at %s" % time_stamp
		rospy.loginfo(hello_str)
		### get the image
		rgbImg = camera.takeImage(servers[1], saveImages, time_stamp)
		### convert the image format to ros message
		rgb_msg = CvBridge().cv2_to_imgmsg(rgbImg)
		rgbImg_pub.publish(rgb_msg)
		rate.sleep()

	# pos, quat = p.getBasePositionAndOrientation(object_pose.m, servers[1])
	# print("pos: " + str(pos))
	# print("quat: " + str(quat))

	time.sleep(3)