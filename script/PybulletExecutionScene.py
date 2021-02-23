#!/usr/bin/env python
from __future__ import division

import pybullet as p
import pybullet_data

import time
import sys
import os
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

# import pkgutil
# egl = pkgutil.get_loader('eglRenderer')

import utils
from MotomanRobot import MotomanRobot
from WorkspaceTable import WorkspaceTable
from SimulatedCamera import SimulatedCamera
from Executor import Executor

import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from pybullet_motoman.srv import ExecuteTrajectory, ExecuteTrajectoryResponse
from pybullet_motoman.msg import EEPoses
from pybullet_motoman.srv import AttachObject, AttachObjectResponse
from pybullet_motoman.srv import EnablePhysics, EnablePhysicsResponse
from pybullet_motoman.srv import AddVirtualBox, AddVirtualBoxResponse

### This class defines a PybulletExecutionScene class which
### (1) sets up the robot, table and camera
### (2) selects the object specified and drop it in the table to get a random object pose

class PybulletExecutionScene(object):

    def __init__(self, args):

        ### read in relevant ros parameters for scene generation
        basePosition, baseOrientation, urdfFile, \
        leftArmHomeConfiguration, rightArmHomeConfiguration, \
        standingBase_dim, table_dim, table_offset_x, transitCenterHeight, \
        camera_extrinsic, camera_intrinsic, object_mesh_path, dropHeight = self.readROSParam()
        rospack = rospkg.RosPack() ### https://wiki.ros.org/Packages
        self.rosPackagePath = rospack.get_path("pybullet_motoman")

        ### set the server for the pybullet real scene
        # self.executingClientID = p.connect(p.DIRECT)
        self.executingClientID = p.connect(p.GUI)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # self.egl_plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
        # print("plugin=", self.egl_plugin)

        ### create an executor assistant
        self.executor_e = Executor(self.executingClientID,
            isObjectInLeftHand=False, isObjectInRightHand=False,
            objectInLeftHand=None, objectInRightHand=None)

        ### configure the robot
        self.configureMotomanRobot(urdfFile, basePosition, baseOrientation, \
                leftArmHomeConfiguration, rightArmHomeConfiguration, True)
        ### set up the workspace
        self.setupWorkspace(standingBase_dim, table_dim, table_offset_x, \
                transitCenterHeight, object_mesh_path, True)
        ### set up the camera
        self.setupCamera(camera_extrinsic, camera_intrinsic, args)

        ### after initialize the scene,
        ### randomize an object in the scene (drop an object on the table)
        self.object_name = args[3]
        # self.workspace_e.dropObjectOnTable(self.object_name, dropHeight)
        self.workspace_e.fixAnObjectOnTable(self.object_name)

    def readROSParam(self):
        ### This functions read in needed ROS parameters
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

        return basePosition, baseOrientation, urdfFile, \
            leftArmHomeConfiguration, rightArmHomeConfiguration, \
            standingBase_dim, table_dim, table_offset_x, transitCenterHeight, \
            camera_extrinsic, camera_intrinsic, object_mesh_path, dropHeight


    def rosInit(self):
        ### This function specifies the role of a node instance for this class ###
        ### and initialize a ros node ###
        ### specify the role of a node instance for this class
        self.color_im_pub = rospy.Publisher('rgb_images', Image, queue_size=10)
        self.depth_im_pub = rospy.Publisher('depth_images', Image, queue_size=10)
        self.jointState_pub = rospy.Publisher("left_right_joint_states", JointState, queue_size=10)
        self.ee_poses_pub = rospy.Publisher('ee_poses', EEPoses, queue_size=10)
        # self.object_pose_pub = rospy.Publisher('object_pose', ObjectPose, queue_size=10)
        execute_trajectory_server = rospy.Service(
                "execute_trajectory", ExecuteTrajectory, self.execute_traj_callback)
        attach_object_server = rospy.Service(
                "attach_object", AttachObject, self.attach_object_callback)
        enable_physics_server = rospy.Service(
                "enable_physics", EnablePhysics, self.enable_physics_callback)
        add_virtual_box_server = rospy.Service(
                "add_virtual_box", AddVirtualBox, self.add_virtual_box_callback)
        rospy.init_node("pybullet_execution_scene", anonymous=True)

    def add_virtual_box_callback(self, req):
        half_extents = np.array(req.dims) / 2
        test_visual = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=half_extents,
                                          rgbaColor=[0, 0, 0, 1])
        _ = p.createMultiBody(baseVisualShapeIndex=test_visual, basePosition=req.position,
                              baseOrientation=req.orientation)
        res = AddVirtualBoxResponse()
        res.success = True
        return res

    def enable_physics_callback(self, req):
        ### given the request data: isPhysicsEnabled (bool)
        if req.isPhysicsEnabled == True:
            self.workspace_e.enablePhysicsEnv()
            print("physics enabled, gravity comes in")
            return EnablePhysicsResponse(True)
        else:
            self.workspace_e.disablePhysicsEnv()
            print("physics turned off")
            return EnablePhysicsResponse(True)


    def attach_object_callback(self, req):
        ### given the request data: attach(bool) + armType
        if req.attach:
            self.executor_e.attachObject(self.workspace_e, self.robot_e, req.armType)
            print("successfully attached the object")
            return AttachObjectResponse(True)
        else:
            self.executor_e.detachObject(self.workspace_e, self.robot_e, req.armType)
            print("successfully detached the object")
            return AttachObjectResponse(True)


    def execute_traj_callback(self, req):
        ### given the request data: traj + armType
        ### execute the trajectory on a specified arm

        trajectory = []
        for edgeConfigs in req.trajectory:
            temp_configs = []
            for joint_state in edgeConfigs.edge_configs:
                temp_configs.append(joint_state.position)
            trajectory.append(temp_configs)

        self.executor_e.executeTrajctory(trajectory, self.robot_e, req.armType)

        return ExecuteTrajectoryResponse(True)


    def configureMotomanRobot(self,
                urdfFile, basePosition, baseOrientation,
                leftArmHomeConfiguration, rightArmHomeConfiguration, isPhysicsTurnOn):
        ### This function configures the robot in the real scene ###
        self.robot_e = MotomanRobot(
            os.path.join(self.rosPackagePath, urdfFile),
            basePosition, baseOrientation, leftArmHomeConfiguration, rightArmHomeConfiguration,
            isPhysicsTurnOn, self.executingClientID)

    def setupWorkspace(self,
            standingBase_dim, table_dim, table_offset_x,
            transitCenterHeight, object_mesh_path, isPhysicsTurnOn):
        ### This function sets up the workspace ###
        self.workspace_e = WorkspaceTable(self.robot_e.basePosition,
            standingBase_dim, table_dim, table_offset_x, transitCenterHeight,
            os.path.join(self.rosPackagePath, object_mesh_path),
            isPhysicsTurnOn, self.executingClientID)

    def setupCamera(self, camera_extrinsic, camera_intrinsic, args):
        ### This function sets up the camera ###
        ### indicate which scene you are working on and whether you want to save images
        self.scene_index = args[1]
        self.saveImages = (args[2] in ('y', 'Y')) ### decide whether to save images or not
        self.camera_e = SimulatedCamera(
            self.workspace_e.tablePosition, self.workspace_e.table_dim,
            camera_extrinsic, camera_intrinsic,
            self.scene_index, self.saveImages,
            self.executingClientID
        )


def main(args):
    pybullet_execution_scene = PybulletExecutionScene(args)
    pybullet_execution_scene.rosInit()
    rate = rospy.Rate(10) ### 10hz
    bridge = CvBridge()

    count = 0

    while not rospy.is_shutdown():
        ### get the time stamp
        time_stamp = rospy.get_time()
        # rospy.loginfo("time stamp for image and joint state publisher %s" % time_stamp)

        ### get current joint state
        motomanRJointNames, armCurrConfiguration = pybullet_execution_scene.robot_e.getJointState()
        ### prepare the message
        joint_state_msg = JointState()
        # joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.header.frame_id = str(time_stamp)
        joint_state_msg.name = motomanRJointNames
        joint_state_msg.position = armCurrConfiguration

        ### publish the message
        # rgbImg, depthImg = pybullet_execution_scene.camera_e.takeRGBImage()
        # rgb_msg = bridge.cv2_to_imgmsg(rgbImg, 'rgb8')
        # depth_msg = bridge.cv2_to_imgmsg((1000 * depthImg).astype(np.uint16), 'mono16')
        # pybullet_execution_scene.color_im_pub.publish(rgb_msg)
        # pybullet_execution_scene.depth_im_pub.publish(depth_msg)
        # if count == 0:
        #     cv2.imwrite(os.path.expanduser('~/color.png'), cv2.cvtColor(rgbImg, cv2.COLOR_RGB2BGR))
        #     cv2.imwrite(os.path.expanduser('~/depth.png'), (1000 * depthImg).astype(np.uint16))

        pybullet_execution_scene.jointState_pub.publish(joint_state_msg)
        ee_poses_msgs = pybullet_execution_scene.executor_e.prepare_ee_poses_msgs(pybullet_execution_scene.robot_e)
        pybullet_execution_scene.ee_poses_pub.publish(ee_poses_msgs)

        count += 1
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
