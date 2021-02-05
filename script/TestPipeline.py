#!/usr/bin/env python
from __future__ import division

import time
import sys
import os

import rospy
import rospkg
from std_msgs.msg import String 
from geometry_msgs.msg import Pose 

from pybullet_motoman.srv import MotionPlanning, MotionPlanningRequest
from pybullet_motoman.srv import AttachObject, AttachObjectRequest

### This program serves as the pipeline which
### coordinates with three main nodes
### (1) execution node
### (2) plan node
### (3) perception node

def shiyang_obtain_gripper_poses_for_left_hand(armType, motionType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test

    planning_requests = [] ### a list of MotionPlanningRequest

    request1 = MotionPlanningRequest()
    request1.gripper_pose.position.x = 0.8
    request1.gripper_pose.position.y = 0.45
    request1.gripper_pose.position.z = 0.66
    request1.gripper_pose.orientation.x = 0.0
    request1.gripper_pose.orientation.y = 0.8
    request1.gripper_pose.orientation.z = 0.0
    request1.gripper_pose.orientation.w = 0.0
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


def shiyang_obtain_gripper_poses_at_transit_center(armType, motionType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test

    ### here I set the transit center height to be (position.z = 0.9)
    ###      I also set the distance from the robot body to the front table 
    ###      (position.x = 0.8) make sure the robot arm will not collide its torso body 
    ### feel free to change their values according to your interest


    planning_requests = []

    request1 = MotionPlanningRequest()
    request1.gripper_pose.position.x = 0.8
    request1.gripper_pose.position.y = 0.0
    request1.gripper_pose.position.z = 0.9
    request1.gripper_pose.orientation.x = 0.0
    request1.gripper_pose.orientation.y = 0.8
    request1.gripper_pose.orientation.z = 0.0
    request1.gripper_pose.orientation.w = 0.0
    request1.armType = armType
    request1.motionType = motionType
    planning_requests.append(request1)

    return planning_requests


def shiyang_obtain_gripper_poses_for_right_hand(armType, motionType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test

    planning_requests = [] ### a list of MotionPlanningRequest

    request1 = MotionPlanningRequest()
    request1.gripper_pose.position.x = 0.8
    request1.gripper_pose.position.y = -0.07
    request1.gripper_pose.position.z = 0.8525
    request1.gripper_pose.orientation.x = 0.0
    request1.gripper_pose.orientation.y = 0.707
    request1.gripper_pose.orientation.z = 0.707
    request1.gripper_pose.orientation.w = 0.0
    request1.armType = armType
    request1.motionType = motionType
    planning_requests.append(request1)

    return planning_requests


def serviceCall_motion_planning(planning_request):
    rospy.wait_for_service("motion_planning")
    try:
        plan = rospy.ServiceProxy('motion_planning', MotionPlanning)
        success = plan(planning_request.gripper_pose, 
                        planning_request.armType, planning_request.motionType)
        return success.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def serviceCall_attachObject(isAttachEnabled, armType):
    rospy.wait_for_service("attach_object")
    request = AttachObjectRequest()
    request.isAttachEnabled = isAttachEnabled
    request.armType = armType
    try:
        attachObject = rospy.ServiceProxy('attach_object', AttachObject)
        success  = attachObject(request.isAttachEnabled, request.armType)
        return success.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def main(args):
    ### declaim its role
    ### It is a master which requests plan node to plan
    ### the plan node will request a service from execute node to execute
    ### it also request attach/detach behavior from execute node
    rospy.init_node("test_pipeline", anonymous=True)

    ### request the service to plan
    planning_requests = shiyang_obtain_gripper_poses_for_left_hand(
                    armType="Left", motionType="transit") ### MotionPlanningRequest[]
    for planning_request in planning_requests:
        plan_success = serviceCall_motion_planning(planning_request)
        print("plan_success: ", plan_success)
        print("\n")
        if plan_success: break

    ## before next plan, we want the object to be attached to the gripper
    attach_success = serviceCall_attachObject(isAttachEnabled=True, armType="Left")

    planning_requests = shiyang_obtain_gripper_poses_at_transit_center(
                    armType="Left", motionType="transfer") ### MotionPlanningRequest[]
    for planning_request in planning_requests:
        plan_success = serviceCall_motion_planning(planning_request)
        if plan_success: break

    ## request the service to plan
    planning_requests = shiyang_obtain_gripper_poses_for_right_hand(
                    armType="Right", motionType="transit") ### MotionPlanningRequest[]
    for planning_request in planning_requests:
        plan_success = serviceCall_motion_planning(planning_request)
        if plan_success: break

    ## before next plan, we want the object to be attached to the gripper
    attach_success = serviceCall_attachObject(isAttachEnabled=True, armType="Right")

    ### detach the object from left hand
    detach_success = serviceCall_attachObject(isAttachEnabled=False, armType="Left")

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)