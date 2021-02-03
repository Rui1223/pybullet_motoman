#!/usr/bin/env python
from __future__ import division

import pybullet as p
import pybullet_data

import time
import sys
import os

import utils
from MotomanRobot import MotomanRobot
from WorkspaceTable import WorkspaceTable
from Planner import Planner

import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from pybullet_motoman.srv import MotionPlanning, MotionPlanningResponse
from pybullet_motoman.srv import ExecuteTrajectory, ExecuteTrajectoryRequest
from pybullet_motoman.msg import ObjectPose

### This class defines a PybulletPlanScene class which
### sets up the robot, table in the planning scene (known geometries)
### updates the object geometry based on percerption
### updates robot configuration based on the information from the simulated execution scene
### performs planning for a query (from current configuration to targe configuration)



class PybulletPlanScene(object):

    def __init__(self, args):

        ### read in relevant ros parameters for scene generation
        basePosition, baseOrientation, urdfFile, object_mesh_path, \
        leftArmHomeConfiguration, rightArmHomeConfiguration, \
        standingBase_dim, table_dim, table_offset_x, transitCenterHeight = self.readROSParam()
        rospack = rospkg.RosPack() ### https://wiki.ros.org/Packages
        self.rosPackagePath = rospack.get_path("pybullet_motoman")

        ### set the server for the pybullet planning scene
        self.planningClientID = p.connect(p.DIRECT)
        # self.planningClientID = p.connect(p.GUI)

        ### create a planner assistant
        self.planner_p = Planner(self.rosPackagePath, self.planningClientID)

        ### configure the robot
        self.configureMotomanRobot(urdfFile, basePosition, baseOrientation, \
                leftArmHomeConfiguration, rightArmHomeConfiguration, False)
        ### set up the workspace
        self.setupWorkspace(standingBase_dim, table_dim, table_offset_x, \
                transitCenterHeight, object_mesh_path)


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

        while not rospy.has_param('/object_in_real_scene/object_mesh_path'):
            rospy.sleep(0.2)
        object_mesh_path = rospy.get_param('/object_in_real_scene/object_mesh_path')

        while not rospy.has_param('/workspace_table/transitCenterHeight'):
            rospy.sleep(0.2)
        transitCenterHeight = rospy.get_param('/workspace_table/transitCenterHeight')

        return basePosition, baseOrientation, urdfFile, object_mesh_path, \
            leftArmHomeConfiguration, rightArmHomeConfiguration, \
            standingBase_dim, table_dim, table_offset_x, transitCenterHeight


    def rosInit(self):
        ### This function specifies the role of a node instance for this class ###
        ### and initialize a ros node ###
        ### specify the role of a node instance for this class
        ### claim the service
        motion_planning_server = rospy.Service("motion_planning", MotionPlanning, self.motion_plan_callback)
        # object_pose_sub = rospy.Subscriber("object_pose", ObjectPose, self.objectPose_callback)
        rospy.init_node("pybullet_plan_scene", anonymous=True)

    # def objectPose_callback(self, data):
    #     # print(data)
    #     # print("do nothing")
    #     pass


    def motion_plan_callback(self, req):

        ### update the object location first
        object_pose_msg = rospy.wait_for_message("object_pose", ObjectPose)
        object_name = object_pose_msg.object_name
        object_pose = [[object_pose_msg.object_pose.position.x, 
                object_pose_msg.object_pose.position.y, object_pose_msg.object_pose.position.z], 
                [object_pose_msg.object_pose.orientation.x, object_pose_msg.object_pose.orientation.y, 
                object_pose_msg.object_pose.orientation.z, object_pose_msg.object_pose.orientation.w]]
        self.workspace_p.updateObjectMesh(object_name, object_pose)


        # ### get the current robot config from real scene by looking at the topic "joint_states"
        # joint_states_msg = rospy.wait_for_message('joint_states', JointState)
        # joint_values = list(joint_states_msg.position)
        # self.robot_p.resetArmConfig(joint_values)
        # self.robot_p.updateSingleArmConfig(joint_values[0:7], "Left")
        # self.robot_p.updateSingleArmConfig(joint_values[7:14], "Right")


        ### analyze the target configuration of the robot given the grasp pose
        armType = req.armType
        targetPose = [[req.gripper_pose.position.x, req.gripper_pose.position.y, req.gripper_pose.position.z], 
            [req.gripper_pose.orientation.x, req.gripper_pose.orientation.y, \
                                        req.gripper_pose.orientation.z, req.gripper_pose.orientation.w]]
        isPoseValid = self.planner_p.checkPoseBasedOnConfig(
            targetPose, self.robot_p, self.workspace_p, armType)
        if not isPoseValid:
            print("this pose is not even valid, let alone motion planning")
            return MotionPlanningResponse(False)
        else:
            print("the pose is valid, we will deal with pre-grasp later")
            print("now proceed to motion planning")

        ### get the current robot config from real scene by looking at the topic "joint_states"
        joint_states_msg = rospy.wait_for_message('joint_states', JointState)
        joint_values = list(joint_states_msg.position)
        self.robot_p.resetArmConfig(joint_values)
        self.robot_p.updateSingleArmConfig(joint_values[0:7], "Left")
        self.robot_p.updateSingleArmConfig(joint_values[7:14], "Right")

        if armType == "Left":
            task_name = "LeftPick"
        else:
            task_name = "RightPick"

        result_path, result_traj = self.planner_p.shortestPathPlanning(
                self.robot_p.left_ee_pose, targetPose,
                task_name, self.robot_p, self.workspace_p, armType)
        print("result path: ", result_path)
        print("result_traj: ", result_traj)

        ## get the current robot config from real scene by looking at the topic "joint_states"
        joint_states_msg = rospy.wait_for_message('joint_states', JointState)
        joint_values = list(joint_states_msg.position)
        self.robot_p.resetArmConfig(joint_values)
        self.robot_p.updateSingleArmConfig(joint_values[0:7], "Left")
        self.robot_p.updateSingleArmConfig(joint_values[7:14], "Right")


        if result_path != []:
            print("the path is successfully found")
            ### Now we need to call a service call to execute the path in the execution scene
            if armType == "Left":
                execute_success = self.serviceCall_execute_trajectory(
                                result_traj, self.robot_p.left_ee_pose, targetPose, armType)
            else:
                execute_success = self.serviceCall_execute_trajectory(
                                result_traj, self.robot_p.right_ee_pose, targetPose, armType)                
            return MotionPlanningResponse(True)
        else:
            print("the path is not successfully found")
            return MotionPlanningResponse(False)

        

    def serviceCall_execute_trajectory(self, result_traj, initialPose, targetPose, armType):
        rospy.wait_for_service("execute_trajectory")
        request = ExecuteTrajectoryRequest()
        request.armType = armType

        initial_pose = Pose()
        initial_pose.position.x = initialPose[0][0]
        initial_pose.position.y = initialPose[0][1]
        initial_pose.position.z = initialPose[0][2]
        initial_pose.orientation.x = initialPose[1][0]
        initial_pose.orientation.y = initialPose[1][1]
        initial_pose.orientation.z = initialPose[1][2]
        initial_pose.orientation.w = initialPose[1][3]
        request.poses.append(initial_pose)

        for waypoint in result_traj:
            temp_pose = Pose()
            temp_pose.position.x = waypoint[0]
            temp_pose.position.y = waypoint[1]
            temp_pose.position.z = waypoint[2]
            temp_pose.orientation.x = waypoint[3]
            temp_pose.orientation.y = waypoint[4]
            temp_pose.orientation.z = waypoint[5]
            temp_pose.orientation.w = waypoint[6]
            request.poses.append(temp_pose)

        target_pose = Pose()
        target_pose.position.x = targetPose[0][0]
        target_pose.position.y = targetPose[0][1]
        target_pose.position.z = targetPose[0][2]
        target_pose.orientation.x = targetPose[1][0]
        target_pose.orientation.y = targetPose[1][1]
        target_pose.orientation.z = targetPose[1][2]
        target_pose.orientation.w = targetPose[1][3]
        request.poses.append(target_pose)

        try:
            executeIt = rospy.ServiceProxy("execute_trajectory", ExecuteTrajectory)
            success = executeIt(request.poses, request.armType)
            return success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


    def configureMotomanRobot(self,
                urdfFile, basePosition, baseOrientation,
                leftArmHomeConfiguration, rightArmHomeConfiguration, isPhysicsTurnOn):
        ### This function configures the robot in the planning scene ###
        self.robot_p = MotomanRobot(
            os.path.join(self.rosPackagePath, urdfFile),
            basePosition, baseOrientation, leftArmHomeConfiguration, rightArmHomeConfiguration,
            isPhysicsTurnOn, self.planningClientID)


    def setupWorkspace(self,
            standingBase_dim, table_dim, table_offset_x,
            transitCenterHeight, object_mesh_path):
        ### The function sets up the workspace
        self.workspace_p = WorkspaceTable(self.robot_p.basePosition,
            standingBase_dim, table_dim, table_offset_x, transitCenterHeight,
            os.path.join(self.rosPackagePath, object_mesh_path),
            self.planningClientID)


def main(args):
    pybullet_plan_scene = PybulletPlanScene(args)
    pybullet_plan_scene.planner_p.loadSamples()
    pybullet_plan_scene.rosInit()
    rate = rospy.Rate(10) ### 10hz

    rospy.spin()



if __name__ == '__main__':
    main(sys.argv)
