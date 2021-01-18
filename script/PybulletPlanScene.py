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

from pybullet_motoman.srv import MotionPlanning, MotionPlanningResponse
from pybullet_motoman.msg import PoseSequence3D, ObjectEstimate3D
from pybullet_motoman.msg import Pose3D, BoundingBox3D
from pybullet_motoman.srv import ExecuteTrajectory, ExecuteTrajectoryRequest

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
        self.rospack = rospkg.RosPack() ### https://wiki.ros.org/Packages

        ### set the server for the pybullet planning scene
        self.planningClientID = p.connect(p.GUI)
        ### create a planner assistant
        self.planner_p = Planner(self.planningClientID)

        ### configure the robot
        self.configureMotomanRobot(urdfFile, basePosition, baseOrientation, \
                leftArmHomeConfiguration, rightArmHomeConfiguration)
        ### set up the workspace
        self.setupWorkspace(standingBase_dim, table_dim, table_offset_x, \
                transitCenterHeight, object_mesh_path)


        self.rosInit() ### initialize a ros node

        raw_input("press enter to continue")


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
        rospy.init_node("pybullet_plan_scene", anonymous=True)


    def motion_plan_callback(self, req):

        ### given the request data: object_estimate
        ### update the object geometry in the plan scene 
        ### (either for target or collision check)
        self.workspace_p.updateObjectGeomeotry_BoundingBox(
            req.object_estimate.pose, req.object_estimate.dim)

        ### analyze the target configuration of the robot given the grasp pose
        ### so far we only care about the best grasp pose
        target_config = self.planner_p.generateConfigFromPose(
            req.grasp_pose_candidates.pose_sequence[0], self.robot_p, self.workspace_p, "Left")

        ### get the current robot config from real scene by looking at the topic "joint_states"
        joint_states_msg = rospy.wait_for_message('joint_states', JointState)
        joint_values = list(joint_states_msg.position)
        self.robot_p.resetArmConfig(joint_values)
        self.robot_p.updateSingleArmConfig(joint_values[0:7], "Left")
        self.robot_p.updateSingleArmConfig(joint_values[7:14], "Right")

        ### with target_config and current arm config, we can send a planning query
        result_path = self.planner_p.shortestPathPlanning(
                            self.robot_p.leftArmCurrConfiguration, target_config, 
                            "LeftPick", self.robot_p, self.workspace_p, "Left")

        if result_path != None:
            print("the path is successfully found")
            ### Now we need to call a service call to execute the path in the execution scene
            execute_success = self.serviceCall_execute_trajectory(result_path, "Left")

        return MotionPlanningResponse(True)


    def serviceCall_execute_trajectory(self, result_path, armType):
        rospy.wait_for_service("execute_trajectory")
        request = ExecuteTrajectoryRequest()
        request.armType.armType = armType
        for waypoint in result_path:
            temp_joint_state = JointState()
            temp_joint_state.position = waypoint
            request.joint_trajectory.append(temp_joint_state)
        try:
            executeIt = rospy.ServiceProxy("execute_trajectory", ExecuteTrajectory)
            success = executeIt(request.joint_trajectory, request.armType)
            return success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


    def configureMotomanRobot(self,
                urdfFile, basePosition, baseOrientation,
                leftArmHomeConfiguration, rightArmHomeConfiguration):
        ### This function configures the robot in the planning scene ###
        self.robot_p = MotomanRobot(
            os.path.join(self.rospack.get_path("pybullet_motoman"), urdfFile), 
            basePosition, baseOrientation, leftArmHomeConfiguration, rightArmHomeConfiguration, 
            self.planningClientID)

    def setupWorkspace(self,
            standingBase_dim, table_dim, table_offset_x, 
            transitCenterHeight, object_mesh_path):
        ### The function sets up the workspace
        self.workspace_p = WorkspaceTable(self.robot_p.basePosition, 
            standingBase_dim, table_dim, table_offset_x, transitCenterHeight,
            os.path.join(self.rospack.get_path("pybullet_motoman"), object_mesh_path),
            self.planningClientID)


def main(args):

    pybullet_plan_scene = PybulletPlanScene(args)
    rate = rospy.Rate(10) ### 10hz



    rospy.spin()



if __name__ == '__main__':
    main(sys.argv)