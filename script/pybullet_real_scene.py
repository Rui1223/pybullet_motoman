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
print(utils.__dict__)
print(utils)
from MotomanRobot import MotomanRobot
from WorkspaceTable import WorkspaceTable
from SimulatedCamera import SimulatedCamera
from MotomanController import MotomanController

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
    rate = rospy.Rate(500) ### 10hz
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


    # configure the controller
    # setup the step function for perception
    def sim_step_func():
        ### get the time stamp
        time_stamp = rospy.get_time()
        #rospy.loginfo("time stamp for image and joint state publisher %s" % time_stamp)

        ### get the image
        #rgbImg = camera_e.takeRGBImage()
        ### get current joint state
        motomanRJointNames, armCurrConfiguration = robot_e.getJointState()

        ### prepare the message
        joint_state_msg = JointState()
        # joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.header.frame_id = str(time_stamp)
        joint_state_msg.name = motomanRJointNames
        joint_state_msg.position = armCurrConfiguration
        ### convert the image format to ros message
        #rgb_msg = CvBridge().cv2_to_imgmsg(rgbImg)

        ### publish the message
        #rgbImg_pub.publish(rgb_msg)
        ### publish the message

        # print('joint state names:')
        # print(motomanRJointNames)
        # print('joint state config:')
        # print(armCurrConfiguration)
        

        jointState_pub.publish(joint_state_msg)


    # reset robot using traj
    import json
    f = open('/home/yinglong/Documents/research/task_motion_planning/infrastructure/motoman_ws/src/pybullet_motoman/script/traj1.json', 'r')
    traj_dict = json.load(f)
    joint_names = traj_dict[0]['joint'].keys()
    # map from name to id
    joint_name_to_id = {}
    num_joints = p.getNumJoints(robot_e.motomanGEO, executingClientID)
    for i in range(num_joints):
        jointInfo = p.getJointInfo(robot_e.motomanGEO, i, executingClientID)
        joint_name_to_id[jointInfo[1].decode('ascii')] = i
    for name in joint_names:
        j = joint_name_to_id[name]
        p.resetJointState(robot_e.motomanGEO, j, traj_dict[0]['joint'][name], physicsClientId=executingClientID)


    motomanRJointNames, armCurrConfiguration = robot_e.getJointState()

    # this will create the trajectory server for tracking
    rospy.loginfo('starting controller...')
    #p.setTimeStep(0.0001)
    #1/240
    for i in range(1,8):
        p.changeDynamics(robot_e.motomanGEO, i, lateralFriction=0., linearDamping=0., angularDamping=0., \
                            physicsClientId=executingClientID)
    for i in range(11,18):
        p.changeDynamics(robot_e.motomanGEO, i, lateralFriction=0., linearDamping=0., angularDamping=0., \
                            physicsClientId=executingClientID)

    controller_e = MotomanController(sim_step_func, time_step=1/240, robot_id=robot_e.motomanGEO, robot_joints=motomanRJointNames,\
                                     pybullet_client_id=executingClientID)


    while not rospy.is_shutdown():
        if not controller_e.executing:
            # keep the velocity zero
            #print('executing is false...')
            sim_step_func()
            for i in range(1,8):
                p.setJointMotorControl2(robot_e.motomanGEO, i, p.VELOCITY_CONTROL, targetVelocity=0, physicsClientId=executingClientID)
            for i in range(11,18):
                p.setJointMotorControl2(robot_e.motomanGEO, i, p.VELOCITY_CONTROL, targetVelocity=0, physicsClientId=executingClientID)

            p.stepSimulation(executingClientID)

            rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
