#!/usr/bin/env python
from __future__ import division

import pybullet as p
import pybullet_data

import time
import sys
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError

import utils
from MotomanRobot import MotomanRobot
from WorkspaceTable import WorkspaceTable
from SimulatedCamera import SimulatedCamera

import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState

### this file sets up the robot, table and camera
### this file select the object specified and drop it in the table to get a random object pose


def main(args):

    ### read in relevant ros parameters for scene generation
    while not rospy.has_param('/motoman_robot/basePosition'):
        rospy.sleep(0.2)
    basePosition = rospy.get_param('/motoman_robot/basePosition')

    while not rospy.has_param('/motoman_robot/baseOrientation'):
        rospy.sleep(0.2)
    baseOrientation = rospy.get_param('/motoman_robot/baseOrientation')

    while not rospy.has_param('/motoman_robot/urdfFile'):
        rospy.sleep(0.2)
    urdfFile = rospy.get_param('/motoman_robot/urdfFile')

    while not rospy.has_param('/motoman_robot/leftArmHomeConfiguration'):
        rospy.sleep(0.2)
    leftArmHomeConfiguration = rospy.get_param('/motoman_robot/leftArmHomeConfiguration')

    while not rospy.has_param('/motoman_robot/rightArmHomeConfiguration'):
        rospy.sleep(0.2)
    rightArmHomeConfiguration = rospy.get_param('/motoman_robot/rightArmHomeConfiguration')

    while not rospy.has_param('/workspace_table/standingBase_dim'):
        rospy.sleep(0.2)
    standingBase_dim = rospy.get_param('/workspace_table/standingBase_dim')

    while not rospy.has_param('/workspace_table/table_dim'):
        rospy.sleep(0.2)
    table_dim = rospy.get_param('/workspace_table/table_dim')

    while not rospy.has_param('/workspace_table/table_offset_x'):
        rospy.sleep(0.2)
    table_offset_x = rospy.get_param('/workspace_table/table_offset_x')

    while not rospy.has_param('/workspace_table/transitCenterHeight'):
        rospy.sleep(0.2)
    transitCenterHeight = rospy.get_param('/workspace_table/transitCenterHeight')

    while not rospy.has_param('/simulated_camera/camera_extrinsic'):
        rospy.sleep(0.2)
    camera_extrinsic = rospy.get_param('/simulated_camera/camera_extrinsic')

    while not rospy.has_param('/simulated_camera/camera_intrinsic'):
        rospy.sleep(0.2)
    camera_intrinsic = rospy.get_param('/simulated_camera/camera_intrinsic')

    while not rospy.has_param('/object_in_real_scene/object_mesh_path'):
        rospy.sleep(0.2)
    object_mesh_path = rospy.get_param('/object_in_real_scene/object_mesh_path')

    while not rospy.has_param('/object_in_real_scene/dropHeight'):
        rospy.sleep(0.2)
    dropHeight = rospy.get_param('/object_in_real_scene/dropHeight')


    rgbImg_pub = rospy.Publisher('rgb_images', Image, queue_size=10)
    jointState_pub = rospy.Publisher("joint_states", JointState, queue_size=3)
    rospy.init_node("pybullet_real_scene", anonymous=True)
    rate = rospy.Rate(10) ### 10hz
    rospack = rospkg.RosPack() ### https://wiki.ros.org/Packages

    ### set the server for the pybullet real scene
    executingClientID = p.connect(p.GUI)

    ### configure the robot
    robot_e = MotomanRobot(
        os.path.join(rospack.get_path("pybullet_motoman"), "urdf/motoman.urdf"), 
        basePosition, baseOrientation, leftArmHomeConfiguration, rightArmHomeConfiguration, 
        executingClientID)

    ### configure the workspace
    workspace_e = WorkspaceTable(robot_e.basePosition, 
        standingBase_dim, table_dim, table_offset_x, transitCenterHeight,
        executingClientID)

    ### indicate which scene you are working on and whether you want to save images
    scene_index = sys.argv[1]
    saveImages = (sys.argv[2] in ('y', 'Y')) ### decide whether to save images or not
    camera_e = SimulatedCamera(
        workspace_e.tablePosition, workspace_e.table_dim, 
        camera_extrinsic, camera_intrinsic,
        scene_index, saveImages,
        executingClientID)


    ### randomize an object in the scene (drop an object on the table)
    object_name = sys.argv[3]
    object_mesh = utils.dropObjectOnTable(
            os.path.join(rospack.get_path("pybullet_motoman"), "mesh"), object_name, 
            workspace_e.tablePosition, workspace_e.table_dim, dropHeight,
            executingClientID)


    while not rospy.is_shutdown():
        ### get the time stamp
        time_stamp = rospy.get_time()
        rospy.loginfo("time stamp for image and joint state publisher %s" % time_stamp)

        ### get the image
        rgbImg = camera_e.takeRGBImage()
        ### get current joint state
        motomanRJointNames, armCurrConfiguration = robot_e.getJointState()

        ### prepare the message
        joint_state_msg = JointState()
        # joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.header.frame_id = str(time_stamp)
        joint_state_msg.name = motomanRJointNames
        joint_state_msg.position = armCurrConfiguration
        ### convert the image format to ros message
        rgb_msg = CvBridge().cv2_to_imgmsg(rgbImg)

        ### publish the message
        rgbImg_pub.publish(rgb_msg)
        ### publish the message
        jointState_pub.publish(joint_state_msg)

        
        rate.sleep()


    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
