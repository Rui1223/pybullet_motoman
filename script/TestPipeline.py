#!/usr/bin/env python
from __future__ import division

import time
import sys
import os

import rospy
import rospkg
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from pybullet_motoman.msg import ObjectPoseBox

from pybullet_motoman.srv import MotionPlanning, MotionPlanningRequest
from pybullet_motoman.srv import AttachObject, AttachObjectRequest
from pybullet_motoman.srv import EnablePhysics, EnablePhysicsRequest
from pybullet_motoman.srv import SingleJointChange, SingleJointChangeRequest

### This program serves as the pipeline which
### coordinates with three main nodes
### (1) execution node
### (2) plan node
### (3) perception node


delta_x = 0.1
delta_y = -0.1

# def shiyang_obtain_gripper_poses_for_left_hand(armType, motionType):
def shiyang_obtain_gripper_poses_for_left_hand(
                    table_dim, table_offset_x, armType, motionType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test

    planning_requests = [] ### a list of MotionPlanningRequest

    tablePos_height = 0.0 + (table_dim[2]/2-0.19-0.005)

    request1 = MotionPlanningRequest()
    # request1.gripper_pose.position.x = 0.8 - 0.1 + delta_x
    # request1.gripper_pose.position.y = 0.45 + delta_y
    # request1.gripper_pose.position.z = 0.64 + 0.025 + 0.1 + 0.02 - 0.1 ### hard-coded
    request1.gripper_pose.position.x = 0.8
    request1.gripper_pose.position.y = 0.45
    request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.08 ### hard-coded
    request1.gripper_pose.orientation.x = 0.0
    request1.gripper_pose.orientation.y = 0.8
    request1.gripper_pose.orientation.z = 0.0
    request1.gripper_pose.orientation.w = 0.0
    request1.object_pose.dims = [0.06, 0.16, 0.23]
    # request1.object_pose.position = [0.8-0.1+delta_x, 0.45+delta_y, 0.61 + 0.025 + 0.1 + 0.02 - 0.1]
    request1.object_pose.position = [0.8, 0.45, tablePos_height + table_dim[2]/2 + 0.05]
    request1.object_pose.orientation = [0.0, 0.707, 0.0, 0.707]
    request1.armType = armType
    request1.motionType = motionType
    planning_requests.append(request1)


    return planning_requests


def shiyang_obtain_gripper_poses_for_left_hand_demo_version(
                    table_dim, table_offset_x, armType, motionType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test

    planning_requests = [] ### a list of MotionPlanningRequest

    tablePos_height = 0.0 + (table_dim[2]/2-0.19-0.005)

    request1 = MotionPlanningRequest()
    # request1.gripper_pose.position.x = 0.8 - 0.1 + delta_x
    # request1.gripper_pose.position.y = 0.45 + delta_y
    # request1.gripper_pose.position.z = 0.64 + 0.025 + 0.1 + 0.02 - 0.1 ### hard-coded

    # request1.gripper_pose.position.x = 0.8
    # request1.gripper_pose.position.y = 0.05
    # request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.16 + 0.01 ### hard-coded
    # request1.gripper_pose.orientation.x = 0.0
    # request1.gripper_pose.orientation.y = 1.0
    # request1.gripper_pose.orientation.z = 0.0
    # request1.gripper_pose.orientation.w = 0.0
    # request1.object_pose.dims = [0.06, 0.16, 0.23]
    # # request1.object_pose.position = [0.8-0.1+delta_x, 0.45+delta_y, 0.61 + 0.025 + 0.1 + 0.02 - 0.1]
    # request1.object_pose.position = [0.8, 0.05, tablePos_height + table_dim[2]/2 + 0.09]
    # request1.object_pose.orientation = [0.7071068, 0, 0, 0.7071068]

    request1.gripper_pose.position.x = 0.6800329089164734 + 0.04
    # request1.gripper_pose.position.y = -0.07 - 0.15
    request1.gripper_pose.position.y = -0.08994719386100769 - 0.23/2 + 0.09
    # request1.gripper_pose.position.z = 0.85 + 0.025 - 0.1
    # request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.27
    request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.16 + 0.005
    # request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.2249369812 + 0.1
    request1.gripper_pose.orientation.x = 0.
    request1.gripper_pose.orientation.y = .8
    request1.gripper_pose.orientation.z = 0.0
    request1.gripper_pose.orientation.w = 0.0
    # request1.gripper_pose.orientation.x = 0.0
    # request1.gripper_pose.orientation.y = 0.707
    # request1.gripper_pose.orientation.z = 0.707
    # # request1.gripper_pose.orientation.w = 0.0
    # request1.gripper_pose.orientation.x = 0.5
    # request1.gripper_pose.orientation.y = 0.5
    # request1.gripper_pose.orientation.z = 0.5
    # request1.gripper_pose.orientation.w = -0.5
    request1.object_pose.dims = [0.06, 0.16, 0.23]
    request1.object_pose.position = [0.6800329089164734, -0.08994719386100769 - 0.23/2 + 0.09, tablePos_height + table_dim[2]/2 + 0.16/2 + 0.005]
    request1.object_pose.orientation = [0.7071068, 0, 0, 0.7071068] 
    request1.armType = armType
    request1.motionType = motionType
    planning_requests.append(request1)


    return planning_requests


# def shiyang_obtain_gripper_poses_at_transit_center(
#                     table_dim, table_offset_x, armType, motionType):
#     ### In reality, the pose is provided by Shiyang's perception process
#     ### here is just for a test

#     ### here I set the transit center height to be (position.z = 0.9)
#     ###      I also set the distance from the robot body to the front table
#     ###      (position.x = 0.8) make sure the robot arm will not collide its torso body
#     ### feel free to change their values according to your interest

#     tablePos_height = 0.0 + (table_dim[2]/2-0.19-0.005)

#     planning_requests = []

#     request1 = MotionPlanningRequest()
#     request1.gripper_pose.position.x = 0.8 - 0.1
#     request1.gripper_pose.position.y = 0.0
#     # request1.gripper_pose.position.z = 0.885 + 0.025 - 0.1
#     request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.305
#     request1.gripper_pose.orientation.x = 0.0
#     request1.gripper_pose.orientation.y = 0.8
#     request1.gripper_pose.orientation.z = 0.0
#     request1.gripper_pose.orientation.w = 0.0
#     # request1.object_pose.dims = [0.06, 0.16, 0.23]
#     # request1.object_pose.position = [0.8, 0.45, 0.62]
#     # request1.object_pose.orientation = [0.0, 0.707, 0.0, 0.707]
#     request1.armType = armType
#     request1.motionType = motionType
#     planning_requests.append(request1)

#     return planning_requests


def shiyang_obtain_gripper_poses_at_transit_center_demo_version(
                    table_dim, table_offset_x, armType, motionType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test

    ### here I set the transit center height to be (position.z = 0.9)
    ###      I also set the distance from the robot body to the front table
    ###      (position.x = 0.8) make sure the robot arm will not collide its torso body
    ### feel free to change their values according to your interest

    tablePos_height = 0.0 + (table_dim[2]/2-0.19-0.005)

    planning_requests = []

    request1 = MotionPlanningRequest()
    # request1.gripper_pose.position.x = 0.8 - 0.1 - 0.05
    request1.gripper_pose.position.x = 0.68 - 0.02 - 0.01
    # request1.gripper_pose.position.y = 0.0 - 0.05
    request1.gripper_pose.position.y = 0.0
    # request1.gripper_pose.position.z = 0.885 + 0.025 - 0.1
    # request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.305 + 0.05
    request1.gripper_pose.position.z = 0.88 + 0.05 + 0.05
    request1.gripper_pose.orientation.x = 0.0
    request1.gripper_pose.orientation.y = 1.0
    request1.gripper_pose.orientation.z = 0.0
    request1.gripper_pose.orientation.w = 0.0
    # request1.object_pose.dims = [0.06, 0.16, 0.23]
    # request1.object_pose.position = [0.8, 0.45, 0.62]
    # request1.object_pose.orientation = [0.0, 0.707, 0.0, 0.707]
    request1.armType = armType
    request1.motionType = motionType
    planning_requests.append(request1)

    return planning_requests


# def shiyang_obtain_gripper_poses_for_right_hand(
#                 table_dim, table_offset_x, armType, motionType):
#     ### In reality, the pose is provided by Shiyang's perception process
#     ### here is just for a test

#     planning_requests = [] ### a list of MotionPlanningRequest

#     tablePos_height = 0.0 + (table_dim[2]/2-0.19-0.005)

#     request1 = MotionPlanningRequest()
#     request1.gripper_pose.position.x = 0.8 - 0.1
#     request1.gripper_pose.position.y = -0.07
#     # request1.gripper_pose.position.z = 0.85 + 0.025 - 0.1
#     request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.27
#     request1.gripper_pose.orientation.x = 0.0
#     request1.gripper_pose.orientation.y = 0.707
#     request1.gripper_pose.orientation.z = 0.707
#     request1.gripper_pose.orientation.w = 0.0
#     request1.object_pose.dims = [0.06, 0.16, 0.23]
#     # request1.object_pose.position = [0.79999-0.1, 1.549e-09, 0.84499 + 0.025 - 0.1]
#     request1.object_pose.position = [0.79999, 1.549e-09, tablePos_height + table_dim[2]/2 + 0.26499]
#     request1.object_pose.orientation = [-4.12e-09, 0.707, 3.4397e-09, 0.707]
#     request1.armType = armType
#     request1.motionType = motionType
#     planning_requests.append(request1)

#     return planning_requests


def shiyang_obtain_gripper_poses_for_right_hand_demo_version(
                table_dim, table_offset_x, armType, motionType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test

    planning_requests = [] ### a list of MotionPlanningRequest

    tablePos_height = 0.0 + (table_dim[2]/2-0.19-0.005)
    print("height")
    print(str(tablePos_height + table_dim[2]/2)) ### 0.605

    request1 = MotionPlanningRequest()
    # request1.gripper_pose.position.x = 0.8 - 0.1 - 0.05
    request1.gripper_pose.position.x = 0.6800329089164734 - 0.02 - 0.01
    # request1.gripper_pose.position.y = -0.07 - 0.15
    request1.gripper_pose.position.y = -0.08994719386100769 - 0.23/2 + 0.09 + 0.02
    # request1.gripper_pose.position.z = 0.85 + 0.025 - 0.1
    # request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.27
    request1.gripper_pose.position.z = 0.799929141998291 + 0.05 + 0.05
    # request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.2249369812 + 0.1
    # request1.gripper_pose.orientation.x = 0.0
    # request1.gripper_pose.orientation.y = 0.707
    # request1.gripper_pose.orientation.z = 0.707
    # request1.gripper_pose.orientation.w = 0.0
    request1.gripper_pose.orientation.x = -0.5
    request1.gripper_pose.orientation.y = 0.5
    request1.gripper_pose.orientation.z = 0.5
    request1.gripper_pose.orientation.w = 0.5
    # request1.object_pose.dims = [0.06, 0.16, 0.23]
    # request1.object_pose.position = [0.79999-0.1, 1.549e-09, 0.84499 + 0.025 - 0.1]
    # request1.object_pose.position = [0.7000364065170288, 1.6941397916525602e-05, 0.8299369812011719, tablePos_height + table_dim[2]/2 + 0.26499]
    # request1.object_pose.position = [0.7000364065170288, 1.6941397916525602e-05+0.1, tablePos_height + table_dim[2]/2 + 0.2249369812 + 0.1]
    # [0.7000364065170288, 1.6941397916525602e-05, 0.8299369812011719]
    # request1.object_pose.orientation = [0.7070987224578857, -4.1193296056007966e-05, 9.302276339440141e-06, 0.7071148157119751]
    request1.armType = armType
    request1.motionType = motionType
    planning_requests.append(request1)

    return planning_requests

# def shiyang_obtain_gripper_poses_at_drop_center(
#                     table_dim, table_offset_x, armType, motionType):
#     ### In reality, the pose is provided by Shiyang's perception process
#     ### here is just for a test

#     ### here I set the drop center height to be (position.z = 0.7)
#     ###      I also set the distance from the robot body to the front table
#     ###      (position.x = 0.8) make sure the robot arm will not collide its torso body
#     ### feel free to change their values according to your interest

#     tablePos_height = 0.0 + (table_dim[2]/2-0.19-0.005)

#     planning_requests = []

#     request1 = MotionPlanningRequest()
#     request1.gripper_pose.position.x = 0.8 - 0.1
#     request1.gripper_pose.position.y = 0.0
#     # request1.gripper_pose.position.z = 0.795 + 0.025 + 0.1
#     request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.215
#     request1.gripper_pose.orientation.x = 0.0
#     request1.gripper_pose.orientation.y = 0.8
#     request1.gripper_pose.orientation.z = 0.0
#     request1.gripper_pose.orientation.w = 0.0
#     # request1.object_pose.dims = [0.06, 0.16, 0.23]
#     # request1.object_pose.position = [0.79999, 1.549e-09, 0.85999]
#     # request1.object_pose.orientation = [-4.12e-09, 0.707, 3.4397e-09, 0.707]
#     request1.armType = armType
#     request1.motionType = motionType
#     planning_requests.append(request1)

#     return planning_requests


def shiyang_obtain_gripper_poses_at_drop_center_demo_version(
                    table_dim, table_offset_x, armType, motionType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test

    ### here I set the drop center height to be (position.z = 0.7)
    ###      I also set the distance from the robot body to the front table
    ###      (position.x = 0.8) make sure the robot arm will not collide its torso body
    ### feel free to change their values according to your interest

    # tablePos_height = 0.0 + (table_dim[2]/2-0.19-0.005)

    # planning_requests = []

    # request1 = MotionPlanningRequest()
    # request1.gripper_pose.position.x = 0.6800329089164734 + 0.1
    # request1.gripper_pose.position.y = 0.0
    # # request1.gripper_pose.position.z = 0.795 + 0.025 + 0.1
    # request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.23 + 0.02
    # request1.gripper_pose.orientation.x = -0.7071068 ### turn 180 from positive
    # request1.gripper_pose.orientation.y = 0.7071068
    # request1.gripper_pose.orientation.z = 0.0
    # request1.gripper_pose.orientation.w = 0.0
    # # request1.object_pose.dims = [0.06, 0.16, 0.23]
    # # request1.object_pose.position = [0.79999, 1.549e-09, 0.85999]
    # # request1.object_pose.orientation = [-4.12e-09, 0.707, 3.4397e-09, 0.707]
    # request1.armType = armType
    # request1.motionType = motionType
    # planning_requests.append(request1)

    # return planning_requests
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test

    ### here I set the drop center height to be (position.z = 0.7)
    ###      I also set the distance from the robot body to the front table
    ###      (position.x = 0.8) make sure the robot arm will not collide its torso body
    ### feel free to change their values according to your interest

    tablePos_height = 0.0 + (table_dim[2]/2-0.19-0.005)

    planning_requests = []

    request1 = MotionPlanningRequest()
    request1.gripper_pose.position.x = 0.6800329089164734 + 0.04 + 0.05 - 0.03
    request1.gripper_pose.position.y = -0.08994719386100769 - 0.23/2 + 0.09 + 0.1 + 0.03 + 0.02 + 0.16/2
    # request1.gripper_pose.position.z = 0.795 + 0.025 + 0.1
    request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.23 + 0.05 - 0.03 - 0.01
    request1.gripper_pose.orientation.x = -0.7071068
    request1.gripper_pose.orientation.y = 0.7071068
    request1.gripper_pose.orientation.z = 0.0
    request1.gripper_pose.orientation.w = 0.0

    
    # request1.object_pose.dims = [0.06, 0.16, 0.23]
    # request1.object_pose.position = [0.79999, 1.549e-09, 0.85999]
    # request1.object_pose.orientation = [-4.12e-09, 0.707, 3.4397e-09, 0.707]
    request1.armType = armType
    request1.motionType = motionType
    planning_requests.append(request1)

    return planning_requests


def shiyang_obtain_gripper_poses_for_right_hand_direct_grasp_demo_version(
                table_dim, table_offset_x, armType, motionType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test

    planning_requests = [] ### a list of MotionPlanningRequest

    tablePos_height = 0.0 + (table_dim[2]/2-0.19-0.005)
    print("height")
    print(str(tablePos_height + table_dim[2]/2)) ### 0.605

    request1 = MotionPlanningRequest()
    # request1.gripper_pose.position.x = 0.8 - 0.1 - 0.05
    request1.gripper_pose.position.x = 0.6800329089164734 + 0.03
    # request1.gripper_pose.position.y = -0.07 - 0.15
    request1.gripper_pose.position.y = -0.08994719386100769 - 0.23/2 + 0.09
    # request1.gripper_pose.position.z = 0.85 + 0.025 - 0.1
    # request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.27
    request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.16 + 0.01
    # request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.2249369812 + 0.1
    request1.gripper_pose.orientation.x = 0.7071068
    request1.gripper_pose.orientation.y = 0.7071068
    request1.gripper_pose.orientation.z = 0.0
    request1.gripper_pose.orientation.w = 0.0
    # request1.gripper_pose.orientation.x = 0.0
    # request1.gripper_pose.orientation.y = 0.707
    # request1.gripper_pose.orientation.z = 0.707
    # # request1.gripper_pose.orientation.w = 0.0
    # request1.gripper_pose.orientation.x = 0.5
    # request1.gripper_pose.orientation.y = 0.5
    # request1.gripper_pose.orientation.z = 0.5
    # request1.gripper_pose.orientation.w = -0.5
    request1.object_pose.dims = [0.06, 0.16, 0.23]
    request1.object_pose.position = [0.6800329089164734 + 0.03, -0.08994719386100769 - 0.23/2 + 0.09, tablePos_height + table_dim[2]/2 + 0.16/2 + 0.005]
    request1.object_pose.orientation = [0.7071068, 0, 0, 0.7071068] 
    # request1.object_pose.position = [0.7000364065170288, 1.6941397916525602e-05, 0.8299369812011719, tablePos_height + table_dim[2]/2 + 0.26499]
    # request1.object_pose.position = [0.7000364065170288, 1.6941397916525602e-05+0.1, tablePos_height + table_dim[2]/2 + 0.2249369812 + 0.1]

    # [0.7000364065170288, 1.6941397916525602e-05, 0.8299369812011719]
    # request1.object_pose.orientation = [0.7070987224578857, -4.1193296056007966e-05, 9.302276339440141e-06, 0.7071148157119751]
    request1.armType = armType
    request1.motionType = motionType
    planning_requests.append(request1)

    return planning_requests


def shiyang_obtain_gripper_poses_at_drop_center_right_hand_direct_grasp_demo_version(
                    table_dim, table_offset_x, armType, motionType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test

    ### here I set the drop center height to be (position.z = 0.7)
    ###      I also set the distance from the robot body to the front table
    ###      (position.x = 0.8) make sure the robot arm will not collide its torso body
    ### feel free to change their values according to your interest

    tablePos_height = 0.0 + (table_dim[2]/2-0.19-0.005)

    planning_requests = []

    request1 = MotionPlanningRequest()
    request1.gripper_pose.position.x = 0.6800329089164734 + 0.04 + 0.05
    request1.gripper_pose.position.y = -0.08994719386100769 - 0.23/2 + 0.09 + 0.1 + 0.03 + 0.02
    # request1.gripper_pose.position.z = 0.795 + 0.025 + 0.1
    request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.23/2 + 0.05 - 0.03
    request1.gripper_pose.orientation.x = -0.5
    request1.gripper_pose.orientation.y = 0.5
    request1.gripper_pose.orientation.z = 0.5
    request1.gripper_pose.orientation.w = 0.5

    
    # request1.object_pose.dims = [0.06, 0.16, 0.23]
    # request1.object_pose.position = [0.79999, 1.549e-09, 0.85999]
    # request1.object_pose.orientation = [-4.12e-09, 0.707, 3.4397e-09, 0.707]
    request1.armType = armType
    request1.motionType = motionType
    planning_requests.append(request1)

    return planning_requests



def serviceCall_motion_planning(planning_request):
    rospy.wait_for_service("motion_planning")
    try:
        plan = rospy.ServiceProxy('motion_planning', MotionPlanning)
        success = plan(planning_request.gripper_pose, planning_request.object_pose,
                        planning_request.armType, planning_request.motionType)
        return success.success
    except rospy.ServiceException as e:
        print("motion_planning service call failed: %s" % e)


def serviceCall_attachObject(attach, armType):
    rospy.wait_for_service("attach_object")
    request = AttachObjectRequest()
    request.attach = attach
    request.armType = armType
    try:
        attachObject = rospy.ServiceProxy('attach_object', AttachObject)
        success  = attachObject(request.attach, request.armType)
        return success.success
    except rospy.ServiceException as e:
        print("attach_object service call failed: %s" % e)


def serviceCall_singleJointChange(rotate_angle, joint_name, armType):
    rospy.wait_for_service("single_joint_change")
    request = SingleJointChangeRequest()
    request.rotate_angle = rotate_angle
    request.joint_name = joint_name
    request.armType = armType
    try:
        singleJointChange = rospy.ServiceProxy('single_joint_change', SingleJointChange)
        success = singleJointChange(request.rotate_angle, request.joint_name, request.armType)
        return success.success
    except rospy.ServiceException as e:
        print("single_joint_change service call failed: %s" % e)


def serviceCall_enablePhysics(isPhysicsEnabled):
    rospy.wait_for_service("enable_physics")
    request = EnablePhysicsRequest()
    request.isPhysicsEnabled = isPhysicsEnabled
    try:
        enablePhysics = rospy.ServiceProxy('enable_physics', EnablePhysics)
        success = enablePhysics(request.isPhysicsEnabled)
        return success.success
    except rospy.ServiceException as e:
        print("enable_physics service call failed: %s" % e)


def readTableInfo():
    while not rospy.has_param('/workspace_table/table_dim'):
        rospy.sleep(0.2)
    table_dim = rospy.get_param('/workspace_table/table_dim')

    while not rospy.has_param('/workspace_table/table_offset_x'):
        rospy.sleep(0.2)
    table_offset_x = rospy.get_param('/workspace_table/table_offset_x')

    return table_dim, table_offset_x


def rightHandDirectGrasp():
    planning_requests = shiyang_obtain_gripper_poses_for_right_hand_direct_grasp_demo_version(
        table_dim, table_offset_x, armType="Right", motionType="transit") ### MotionPlanningRequest[]
    for planning_request in planning_requests:
        plan_success = serviceCall_motion_planning(planning_request)
        print("plan_success: ", plan_success)
        print("\n")
        if plan_success: break
    
    # time.sleep(100000)

    attach_success = serviceCall_attachObject(attach=True, armType="Right")


    planning_requests = shiyang_obtain_gripper_poses_at_drop_center_right_hand_direct_grasp_demo_version(
        table_dim, table_offset_x, armType="Right", motionType="transfer") ### MotionPlanningRequest[]
    for planning_request in planning_requests:
        plan_success = serviceCall_motion_planning(planning_request)
        print("plan_success: ", plan_success)
        print("\n")
        if plan_success: break

    # time.sleep(100000)
    detach_success = serviceCall_attachObject(attach=False, armType="Right")
    # enable_physics_success = serviceCall_enablePhysics(isPhysicsEnabled=True)
    time.sleep(0.3)
    # distable_physics_success = serviceCall_enablePhysics(isPhysicsEnabled=False)

    planning_request = MotionPlanningRequest()
    planning_request.armType = "Right"
    planning_request.motionType = "moveAway"
    # planning_request.object_pose.dims = [0.06, 0.16, 0.23]
    # planning_request.object_pose.position = [0.79999, 1.549e-09, 0.85999]
    # planning_request.object_pose.orientation = [-4.12e-09, 0.707, 3.4397e-09, 0.707]
    plan_success = serviceCall_motion_planning(planning_request)
    print("plan_success: ", plan_success)
    print("\n")


def leftHandGrasp():
    ## request the service to plan
    planning_requests = shiyang_obtain_gripper_poses_for_left_hand_demo_version(
        table_dim, table_offset_x, armType="Left", motionType="transit") ### MotionPlanningRequest[]
    for planning_request in planning_requests:
        plan_success = serviceCall_motion_planning(planning_request)
        print("plan_success: ", plan_success)
        print("\n")
        if plan_success: break
    
    # time.sleep(1000000)

    ## before next plan, we want the object to be attached to the gripper
    attach_success = serviceCall_attachObject(attach=True, armType="Left")

    ### Now move the object at the transit center
    planning_requests = shiyang_obtain_gripper_poses_at_transit_center_demo_version(
        table_dim, table_offset_x, armType="Left", motionType="transfer") ### MotionPlanningRequest[]
    for planning_request in planning_requests:
        plan_success = serviceCall_motion_planning(planning_request)
        print("plan_success: ", plan_success)
        print("\n")
        if plan_success: break

    # time.sleep(100000)
    

    ## request the service to plan
    planning_requests = shiyang_obtain_gripper_poses_for_right_hand_demo_version(
        table_dim, table_offset_x, armType="Right", motionType="transit") ### MotionPlanningRequest[]
    for planning_request in planning_requests:
        plan_success = serviceCall_motion_planning(planning_request)
        print("plan_success: ", plan_success)
        print("\n")
        if plan_success: break
    
    # time.sleep(1000000)
    
    ## before next plan, we want the object to be attached to the gripper
    attach_success = serviceCall_attachObject(attach=True, armType="Right")

    
    ### detach the object from left hand
    detach_success = serviceCall_attachObject(attach=False, armType="Left")

    ### reset left arm (left arm has finished its work)
    planning_request = MotionPlanningRequest()
    planning_request.armType = "Left"
    planning_request.motionType = "moveAway"
    # planning_request.object_pose.dims = [0.06, 0.16, 0.23]
    # planning_request.object_pose.position = [0.79999, 1.549e-09, 0.85999]
    # planning_request.object_pose.orientation = [-4.12e-09, 0.707, 3.4397e-09, 0.707]
    plan_success = serviceCall_motion_planning(planning_request)
    print("plan_success: ", plan_success)
    print("\n")
    
    # time.sleep(1000000)
    
    ### Now move the object at the drop center
    planning_requests = shiyang_obtain_gripper_poses_at_drop_center_demo_version(
        table_dim, table_offset_x, armType="Right", motionType="transfer") ### MotionPlanningRequest[]
    for planning_request in planning_requests:
        plan_success = serviceCall_motion_planning(planning_request)
        print("plan_success: ", plan_success)
        print("\n")
        if plan_success: break

    # time.sleep(100000)

    ## approach to placement
    # planning_request = MotionPlanningRequest()
    # planning_request.armType = "Right"
    # planning_request.motionType = "approachToPlacement"
    # plan_success = serviceCall_motion_planning(planning_request)
    # print("plan_success: ", plan_success)
    # print("\n")

    ### drop the object ###
    ### first detach the object from left hand
    detach_success = serviceCall_attachObject(attach=False, armType="Right")
    time.sleep(1)
    # enable_physics_success = serviceCall_enablePhysics(isPhysicsEnabled=True)
    # time.sleep(1)
    # distable_physics_success = serviceCall_enablePhysics(isPhysicsEnabled=False)

    ### reset the arm to leave the object
    planning_request = MotionPlanningRequest()
    planning_request.armType = "Right"
    planning_request.motionType = "moveAway"
    # planning_request.object_pose.dims = [0.06, 0.16, 0.23]
    # planning_request.object_pose.position = [0.79999, 1.549e-09, 0.85999]
    # planning_request.object_pose.orientation = [-4.12e-09, 0.707, 3.4397e-09, 0.707]
    plan_success = serviceCall_motion_planning(planning_request)


if __name__ == '__main__':
    ### declaim its role
    ### It is a master which requests plan node to plan
    ### the plan node will request a service from execute node to execute
    ### it also request attach/detach behavior from execute node
    table_dim, table_offset_x = readTableInfo()
    rospy.init_node("test_pipeline", anonymous=True)

    # ### rotation test
    # # rotate_angle = 160
    # # single_joint_change_success = serviceCall_singleJointChange(
    # #                     rotate_angle, joint_name="arm_right_joint_7_t", armType="Right")
    
    # rightHandDirectGrasp()
    leftHandGrasp()

    time.sleep(10000)

    '''
    ## request the service to plan
    planning_requests = shiyang_obtain_gripper_poses_for_left_hand_demo_version(
        table_dim, table_offset_x, armType="Left", motionType="transit") ### MotionPlanningRequest[]
    for planning_request in planning_requests:
        plan_success = serviceCall_motion_planning(planning_request)
        print("plan_success: ", plan_success)
        print("\n")
        if plan_success: break
    
    # time.sleep(1000000)

    ## before next plan, we want the object to be attached to the gripper
    attach_success = serviceCall_attachObject(attach=True, armType="Left")

    ### Now move the object at the transit center
    planning_requests = shiyang_obtain_gripper_poses_at_transit_center_demo_version(
        table_dim, table_offset_x, armType="Left", motionType="transfer") ### MotionPlanningRequest[]
    for planning_request in planning_requests:
        plan_success = serviceCall_motion_planning(planning_request)
        print("plan_success: ", plan_success)
        print("\n")
        if plan_success: break

    # time.sleep(100000)
    

    ## request the service to plan
    planning_requests = shiyang_obtain_gripper_poses_for_right_hand_demo_version(
        table_dim, table_offset_x, armType="Right", motionType="transit") ### MotionPlanningRequest[]
    for planning_request in planning_requests:
        plan_success = serviceCall_motion_planning(planning_request)
        print("plan_success: ", plan_success)
        print("\n")
        if plan_success: break
    
    # time.sleep(1000000)
    
    ## before next plan, we want the object to be attached to the gripper
    attach_success = serviceCall_attachObject(attach=True, armType="Right")

    
    ### detach the object from left hand
    detach_success = serviceCall_attachObject(attach=False, armType="Left")

    ### reset left arm (left arm has finished its work)
    planning_request = MotionPlanningRequest()
    planning_request.armType = "Left"
    planning_request.motionType = "moveAway"
    # planning_request.object_pose.dims = [0.06, 0.16, 0.23]
    # planning_request.object_pose.position = [0.79999, 1.549e-09, 0.85999]
    # planning_request.object_pose.orientation = [-4.12e-09, 0.707, 3.4397e-09, 0.707]
    plan_success = serviceCall_motion_planning(planning_request)
    print("plan_success: ", plan_success)
    print("\n")
    
    # time.sleep(1000000)
    
    ### Now move the object at the drop center
    planning_requests = shiyang_obtain_gripper_poses_at_drop_center_demo_version(
        table_dim, table_offset_x, armType="Right", motionType="transfer") ### MotionPlanningRequest[]
    for planning_request in planning_requests:
        plan_success = serviceCall_motion_planning(planning_request)
        print("plan_success: ", plan_success)
        print("\n")
        if plan_success: break

    # time.sleep(100000)

    ### approach to placement
    planning_request = MotionPlanningRequest()
    planning_request.armType = "Right"
    planning_request.motionType = "approachToPlacement"
    plan_success = serviceCall_motion_planning(planning_request)
    print("plan_success: ", plan_success)
    print("\n")

    ### drop the object ###
    ### first detach the object from left hand
    detach_success = serviceCall_attachObject(attach=False, armType="Right")
    time.sleep(1)
    enable_physics_success = serviceCall_enablePhysics(isPhysicsEnabled=True)
    time.sleep(1)
    distable_physics_success = serviceCall_enablePhysics(isPhysicsEnabled=False)

    ### reset the arm to leave the object
    planning_request = MotionPlanningRequest()
    planning_request.armType = "Right"
    planning_request.motionType = "reset"
    # planning_request.object_pose.dims = [0.06, 0.16, 0.23]
    # planning_request.object_pose.position = [0.79999, 1.549e-09, 0.85999]
    # planning_request.object_pose.orientation = [-4.12e-09, 0.707, 3.4397e-09, 0.707]
    plan_success = serviceCall_motion_planning(planning_request)
    '''
    
    time.sleep(10000)
