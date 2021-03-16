#!/usr/bin/env python
from __future__ import division

import time
import sys
import os

import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image

from pybullet_motoman.srv import MotionPlanning, MotionPlanningRequest
from pybullet_motoman.msg import PoseSequence3D, ObjectEstimate3D
from pybullet_motoman.msg import Pose3D, BoundingBox3D


### this program performs pose estimation for the object in the image (subscribe to 'rgb_images' topic)
### this program provides the service the the planning node to 
### send the robot grasp pose, object global pose, and the bounding box


def imageProcess_callback(data):
    pass


def perception_code(image_msg):
    ### this will be replaced by Shiyang's code
    print("performing perception process\n")

    ### get robot grasp pose
    poseInfo = MotionPlanningRequest()
    grasp_pose1 = Pose3D()
    grasp_pose1.position.x = 0.8
    grasp_pose1.position.y = 0.45
    grasp_pose1.position.z = 0.68
    grasp_pose1.orientation.x = 0.0
    grasp_pose1.orientation.y = 0.8
    grasp_pose1.orientation.z = 0.0
    grasp_pose1.orientation.w = 0.0
    poseInfo.grasp_pose_candidates.pose_sequence.append(grasp_pose1)
    grasp_pose2 = Pose3D()
    grasp_pose2.position.x = 0.3
    grasp_pose2.position.y = 0.6
    grasp_pose2.position.z = 0.68
    grasp_pose2.orientation.x = 0.0
    grasp_pose2.orientation.y = 1.0
    grasp_pose2.orientation.z = 0.0
    grasp_pose2.orientation.w = 0.0
    poseInfo.grasp_pose_candidates.pose_sequence.append(grasp_pose2)

    temp_object_pose = Pose3D()
    temp_object_pose.position.x = 0.80
    temp_object_pose.position.y = 0.45
    temp_object_pose.position.z = 0.64
    temp_object_pose.orientation.x = 0.0
    temp_object_pose.orientation.y = 0.8
    temp_object_pose.orientation.z = 0.0
    temp_object_pose.orientation.w = 0.0
    temp_bounding_box = BoundingBox3D(0.2, 0.15, 0.06)

    poseInfo.object_estimate.pose = temp_object_pose
    poseInfo.object_estimate.dim = temp_bounding_box

    return poseInfo


def perception_test():
    ### this will be replaced by Shiyang's code
    print("performing perception process\n")

    ### get robot grasp pose
    poseInfo = MotionPlanningRequest()
    grasp_pose1 = Pose3D()
    grasp_pose1.position.x = 0.8
    grasp_pose1.position.y = 0.45
    grasp_pose1.position.z = 0.68
    grasp_pose1.orientation.x = 0.0
    grasp_pose1.orientation.y = 0.8
    grasp_pose1.orientation.z = 0.0
    grasp_pose1.orientation.w = 0.0
    poseInfo.grasp_pose_candidates.pose_sequence.append(grasp_pose1)
    grasp_pose2 = Pose3D()
    grasp_pose2.position.x = 0.3
    grasp_pose2.position.y = 0.6
    grasp_pose2.position.z = 0.68
    grasp_pose2.orientation.x = 0.0
    grasp_pose2.orientation.y = 1.0
    grasp_pose2.orientation.z = 0.0
    grasp_pose2.orientation.w = 0.0
    poseInfo.grasp_pose_candidates.pose_sequence.append(grasp_pose2)

    temp_object_pose = Pose3D()
    temp_object_pose.position.x = 0.80
    temp_object_pose.position.y = 0.45
    temp_object_pose.position.z = 0.64
    temp_object_pose.orientation.x = 0.0
    temp_object_pose.orientation.y = 0.8
    temp_object_pose.orientation.z = 0.0
    temp_object_pose.orientation.w = 0.0
    temp_bounding_box = BoundingBox3D(0.2, 0.15, 0.06)

    poseInfo.object_estimate.pose = temp_object_pose
    poseInfo.object_estimate.dim = temp_bounding_box

    return poseInfo


def perception_test1():
    ### this will be replaced by Shiyang's code
    print("performing perception process\n")

    ### get robot grasp pose
    poseInfo = MotionPlanningRequest()
    grasp_pose1 = Pose3D()
    grasp_pose1.position.x = 0.8
    grasp_pose1.position.y = 0.0
    grasp_pose1.position.z = 0.9925
    grasp_pose1.orientation.x = 0.0
    grasp_pose1.orientation.y = 0.8
    grasp_pose1.orientation.z = 0.0
    grasp_pose1.orientation.w = 0.0
    poseInfo.grasp_pose_candidates.pose_sequence.append(grasp_pose1)
    grasp_pose2 = Pose3D()
    grasp_pose2.position.x = 0.3
    grasp_pose2.position.y = 0.6
    grasp_pose2.position.z = 0.68
    grasp_pose2.orientation.x = 0.0
    grasp_pose2.orientation.y = 1.0
    grasp_pose2.orientation.z = 0.0
    grasp_pose2.orientation.w = 0.0
    poseInfo.grasp_pose_candidates.pose_sequence.append(grasp_pose2)

    temp_object_pose = Pose3D()
    temp_object_pose.position.x = 0.80
    temp_object_pose.position.y = 0.45
    temp_object_pose.position.z = 0.64
    temp_object_pose.orientation.x = 0.0
    temp_object_pose.orientation.y = 0.8
    temp_object_pose.orientation.z = 0.0
    temp_object_pose.orientation.w = 0.0
    temp_bounding_box = BoundingBox3D(0.2, 0.15, 0.06)

    poseInfo.object_estimate.pose = temp_object_pose
    poseInfo.object_estimate.dim = temp_bounding_box

    return poseInfo


def serviceCall_motion_planning(poseInfo):
    rospy.wait_for_service("motion_planning")
    try:
        plan = rospy.ServiceProxy('motion_planning', MotionPlanning)
        success = plan(poseInfo.grasp_pose_candidates, poseInfo.object_estimate)
        return success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)



def main(args):

    rospy.init_node("fake_perception", anonymous=True)

    rgbImg_sub = rospy.Subscriber("rgb_images", Image, imageProcess_callback)

    time.sleep(5)

    ### prepare the request data for the motion planning service
    # image_msg = rospy.wait_for_message('rgb_images', Image)
    # poseInfo = perception_code(image_msg)
    poseInfo = perception_test()
    ### now we have the request data, now call the service
    plan_success = serviceCall_motion_planning(poseInfo)
    if plan_success != None:
        print("planning is success? ", plan_success)

    print("Second planning\n\n\n")

    poseInfo = perception_test1()
    ### now we have the request data, now call the service
    plan_success = serviceCall_motion_planning(poseInfo)
    if plan_success != None:
        print("planning is success? ", plan_success)    


    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)