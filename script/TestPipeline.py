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

def shiyang_obtain_gripper_poses_for_left_hand(
                    table_dim, table_offset_x, armType, motionType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test

    planning_requests = [] ### a list of MotionPlanningRequest

    tablePos_height = 0.0 + (table_dim[2]/2-0.19-0.005)

    request1 = MotionPlanningRequest()
    request1.gripper_pose.position.x = 0.8
    request1.gripper_pose.position.y = 0.45
    request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.06 ### hard-coded
    request1.gripper_pose.orientation.x = 0.0
    request1.gripper_pose.orientation.y = 0.8
    request1.gripper_pose.orientation.z = 0.0
    request1.gripper_pose.orientation.w = 0.0
    request1.object_pose.dims = [0.06, 0.16, 0.23]
    request1.object_pose.position = [0.8, 0.45, tablePos_height + table_dim[2]/2 + 0.03]
    request1.object_pose.orientation = [0.0, 0.707, 0.0, 0.707]
    request1.armType = armType
    request1.motionType = motionType
    planning_requests.append(request1)

    # request2 = MotionPlanningRequest()
    # request2.gripper_pose.position.x = 0.8
    # request2.gripper_pose.position.y = 0.45
    # request2.gripper_pose.position.z = 0.69
    # request2.gripper_pose.orientation.x = 0.0
    # request2.gripper_pose.orientation.y = 0.8
    # request2.gripper_pose.orientation.z = 0.0
    # request2.gripper_pose.orientation.w = 0.0
    # request2.armType = armType
    # request2.motionType = motionType
    # planning_requests.append(request2)

    return planning_requests


def shiyang_obtain_gripper_poses_at_transit_center(
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
    request1.gripper_pose.position.x = 0.8
    request1.gripper_pose.position.y = 0.0
    request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.305
    request1.gripper_pose.orientation.x = 0.0
    request1.gripper_pose.orientation.y = 0.8
    request1.gripper_pose.orientation.z = 0.0
    request1.gripper_pose.orientation.w = 0.0
    # request1.object_pose.dims = [0.06, 0.16, 0.23]
    # request1.object_pose.position = [0.8, 0.45, 0.62]
    # request1.object_pose.orientation = [0.0, 0.707, 0.0, 0.707]
    request1.armType = armType
    request1.motionType = motionType
    planning_requests.append(request1)

    return planning_requests


def shiyang_obtain_gripper_poses_for_right_hand(
                table_dim, table_offset_x, armType, motionType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test

    planning_requests = [] ### a list of MotionPlanningRequest

    tablePos_height = 0.0 + (table_dim[2]/2-0.19-0.005)

    request1 = MotionPlanningRequest()
    request1.gripper_pose.position.x = 0.8
    request1.gripper_pose.position.y = -0.07
    request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.27
    request1.gripper_pose.orientation.x = 0.0
    request1.gripper_pose.orientation.y = 0.707
    request1.gripper_pose.orientation.z = 0.707
    request1.gripper_pose.orientation.w = 0.0
    request1.object_pose.dims = [0.06, 0.16, 0.23]
    request1.object_pose.position = [0.79999, 1.549e-09, tablePos_height + table_dim[2]/2 + 0.26499]
    request1.object_pose.orientation = [-4.12e-09, 0.707, 3.4397e-09, 0.707]
    request1.armType = armType
    request1.motionType = motionType
    planning_requests.append(request1)

    return planning_requests


def shiyang_obtain_gripper_poses_at_drop_center(
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
    request1.gripper_pose.position.x = 0.8
    request1.gripper_pose.position.y = 0.0
    request1.gripper_pose.position.z = tablePos_height + table_dim[2]/2 + 0.215
    request1.gripper_pose.orientation.x = 0.0
    request1.gripper_pose.orientation.y = 0.8
    request1.gripper_pose.orientation.z = 0.0
    request1.gripper_pose.orientation.w = 0.0
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

    # ## request the service to plan
    # planning_requests = shiyang_obtain_gripper_poses_for_left_hand(
    #     table_dim, table_offset_x, armType="Left", motionType="transit") ### MotionPlanningRequest[]
    # for planning_request in planning_requests:
    #     plan_success = serviceCall_motion_planning(planning_request)
    #     print("plan_success: ", plan_success)
    #     print("\n")
    #     if plan_success: break


    # ## before next plan, we want the object to be attached to the gripper
    # attach_success = serviceCall_attachObject(attach=True, armType="Left")

    # ### Now move the object at the transit center
    # planning_requests = shiyang_obtain_gripper_poses_at_transit_center(
    #     table_dim, table_offset_x, armType="Left", motionType="transfer") ### MotionPlanningRequest[]
    # for planning_request in planning_requests:
    #     plan_success = serviceCall_motion_planning(planning_request)
    #     print("plan_success: ", plan_success)
    #     print("\n")
    #     if plan_success: break
    
    # ## request the service to plan
    # planning_requests = shiyang_obtain_gripper_poses_for_right_hand(
    #     table_dim, table_offset_x, armType="Right", motionType="transit") ### MotionPlanningRequest[]
    # for planning_request in planning_requests:
    #     plan_success = serviceCall_motion_planning(planning_request)
    #     print("plan_success: ", plan_success)
    #     print("\n")
    #     if plan_success: break
    
    # ## before next plan, we want the object to be attached to the gripper
    # attach_success = serviceCall_attachObject(attach=True, armType="Right")

    # ### detach the object from left hand
    # detach_success = serviceCall_attachObject(attach=False, armType="Left")

    # ### reset left arm (left arm has finished its work)
    # planning_request = MotionPlanningRequest()
    # planning_request.armType = "Left"
    # planning_request.motionType = "moveAway"
    # # planning_request.object_pose.dims = [0.06, 0.16, 0.23]
    # # planning_request.object_pose.position = [0.79999, 1.549e-09, 0.85999]
    # # planning_request.object_pose.orientation = [-4.12e-09, 0.707, 3.4397e-09, 0.707]
    # plan_success = serviceCall_motion_planning(planning_request)
    # print("plan_success: ", plan_success)
    # print("\n")

    # ### Now move the object at the drop center
    # planning_requests = shiyang_obtain_gripper_poses_at_drop_center(
    #     table_dim, table_offset_x, armType="Right", motionType="transfer") ### MotionPlanningRequest[]
    # for planning_request in planning_requests:
    #     plan_success = serviceCall_motion_planning(planning_request)
    #     print("plan_success: ", plan_success)
    #     print("\n")
    #     if plan_success: break

    # ### approach to placement
    # planning_request = MotionPlanningRequest()
    # planning_request.armType = "Right"
    # planning_request.motionType = "approachToPlacement"
    # plan_success = serviceCall_motion_planning(planning_request)
    # print("plan_success: ", plan_success)
    # print("\n")

    # ### drop the object ###
    # ### first detach the object from left hand
    # detach_success = serviceCall_attachObject(attach=False, armType="Right")
    # time.sleep(1)
    # enable_physics_success = serviceCall_enablePhysics(isPhysicsEnabled=True)
    # time.sleep(1)
    # distable_physics_success = serviceCall_enablePhysics(isPhysicsEnabled=False)

    # ### reset the arm to leave the object
    # planning_request = MotionPlanningRequest()
    # planning_request.armType = "Right"
    # planning_request.motionType = "reset"
    # # planning_request.object_pose.dims = [0.06, 0.16, 0.23]
    # # planning_request.object_pose.position = [0.79999, 1.549e-09, 0.85999]
    # # planning_request.object_pose.orientation = [-4.12e-09, 0.707, 3.4397e-09, 0.707]
    # plan_success = serviceCall_motion_planning(planning_request)
    
    time.sleep(10000)
