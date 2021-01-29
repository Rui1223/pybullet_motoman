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

def prepare_gripper_poses(armType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test
    poseInfo = MotionPlanningRequest()
    grasp_pose1 = Pose()
    grasp_pose1.position.x = 0.8
    grasp_pose1.position.y = 0.45
    grasp_pose1.position.z = 0.68
    grasp_pose1.orientation.x = 0.0
    grasp_pose1.orientation.y = 0.8
    grasp_pose1.orientation.z = 0.0
    grasp_pose1.orientation.w = 0.0
    poseInfo.gripper_pose_candidates.append(grasp_pose1)
    grasp_pose2 = Pose()
    grasp_pose2.position.x = 0.3
    grasp_pose2.position.y = 0.6
    grasp_pose2.position.z = 0.68
    grasp_pose2.orientation.x = 0.0
    grasp_pose2.orientation.y = 1.0
    grasp_pose2.orientation.z = 0.0
    grasp_pose2.orientation.w = 0.0
    poseInfo.gripper_pose_candidates.append(grasp_pose2)

    poseInfo.armType = armType

    return poseInfo


def obtain_gripper_pose_at_transit_center(armType):
    ### In reality, the pose is provided by Shiyang's perception process
    ### here is just for a test
    poseInfo = MotionPlanningRequest()
    grasp_pose1 = Pose()
    grasp_pose1.position.x = 0.8
    grasp_pose1.position.y = 0.0
    grasp_pose1.position.z = 0.9925
    grasp_pose1.orientation.x = 0.0
    grasp_pose1.orientation.y = 0.8
    grasp_pose1.orientation.z = 0.0
    grasp_pose1.orientation.w = 0.0
    poseInfo.gripper_pose_candidates.append(grasp_pose1)

    poseInfo.armType = armType

    return poseInfo


def serviceCall_motion_planning(poseInfo):
    rospy.wait_for_service("motion_planning")
    try:
        plan = rospy.ServiceProxy('motion_planning', MotionPlanning)
        success = plan(poseInfo.gripper_pose_candidates, poseInfo.armType)
        return success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def serviceCall_attachObject(attach, armType):
    rospy.wait_for_service("attach_object")
    request = AttachObjectRequest()
    request.attach = attach
    request.armType = armType
    try:
        attachObject = rospy.ServiceProxy('attach_object', AttachObject)
        success  = attachObject(request.attach, request.armType)
        return success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def main(args):
    ### declaim its role
    ### It is a master which requests plan node to plan
    ### the plan node will request a service from execute node to execute
    ### it also request attach/detach behavior from execute node
    rospy.init_node("test_pipeline", anonymous=True)

    ### request the service to plan
    gripper_poses = prepare_gripper_poses(armType="Left")
    plan_success = serviceCall_motion_planning(gripper_poses)

    ## before next plan, we want the object to be attached to the gripper
    attach_success = serviceCall_attachObject(attach=True, armType="Left")

    new_gripper_pose = obtain_gripper_pose_at_transit_center(armType="Left")
    plan_success = serviceCall_motion_planning(new_gripper_pose)

    ### detach the object to see if it falls
    detach_success = serviceCall_attachObject(attach=False, armType="Right")

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)