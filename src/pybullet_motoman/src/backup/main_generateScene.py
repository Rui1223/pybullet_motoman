from __future__ import division
import pybullet as p
import pybullet_data

import sys
import os
import time
from collections import OrderedDict

import utils
from MotomanRobot import MotomanRobot
from Workspace import Workspace
from Camera import AzureKineticCamera
from MotionPlanner import MotionPlanner
from MotionExecutor import MotionExecutor

exp_mode = "v"
object_name = sys.argv[1]
scene_index = sys.argv[2]
saveImages = (sys.argv[3] in ('y', 'Y')) ### decide whether to save images or not
collectData = (sys.argv[4] in ('y', 'Y')) ### decide whether to collect data or not
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



### cracker_box: 901(tall), 935(middle), 958(low)
### sugar_box: 907(tall), 900(middle), 901(low)
### mustard_bottle: 900(tall)
### pudding_box: 904(middle), 902(low)
### meat_can: 904(tall), 956(low)
### bleach_cleanser: 904(tall), 908(low)

# object_configs_quaternions = {
# 	"003_cracker_box": [[0.719411, 0, 0, 0.694585], [-0.389555, -0.393536, -0.583308, -0.594246], [0.541242, 0.433313, -0.505165, 0.513912]],
# 	"004_sugar_box": [[0.306675, 0, 0, -0.951815], [0.597777, 0.573837, 0.398076, 0.393587], [-0.48375, -0.453115, 0.455718, -0.594134]],
# 	"006_mustard_bottle": [[-0.39829, 0.000556968, 0.00523115, 0.917245]],
# 	"008_pudding_box": [[0.47358, 0.469145, -0.525009, -0.529142], [0.35273, 0, 0, -0.935726]],
# 	"010_potted_meat_can": [[0.666828, 0, 0, 0.745212], [0.596333, 0.594183, -0.382352, -0.380973]],
# 	"021_bleach_cleanser": [[0.941299, 0, 0, -0.337575], [-0.0158242, 0.0155478, -0.708189, 0.705675]] 
# }

object_configs_angles = {
	"003_cracker_box": [[-0.035, -0.336, 87.775], [89.801, -2.119, 112.705], [-25.498, -84.700, 110.177]],
	"004_sugar_box": [[-0.166, -0.100, -144.075], [90.822, -1.909, 67.882], [-7.177, -79.030, 102.698]],
	"006_mustard_bottle": [[0.006, 0.061, -135.114], [87.134, -1.560, 89.805]],
	"008_pudding_box": [[89.426, 0.412, -96.268], [-0.721, 0.300, -138.733]],
	"010_potted_meat_can": [[-0.131, -0.061, 97.479], [87.863, -1.266, -65.330]],
	"021_bleach_cleanser": [[-0.103, -0.082, -39.439], [-84.349, -1.891, -177.925]] 
}
### add an object
object_pose = utils.dropObjectOnTable(object_name, object_configs_angles[object_name], workspace.tablePosition, workspace.table_dim, 0.15, servers[1])
time.sleep(3)
camera.takeImage(servers[1], saveImages, 1)

pos, quat = p.getBasePositionAndOrientation(object_pose.m, servers[1])
print("pos: " + str(pos))
print("quat: " + str(quat))

time.sleep(10000)