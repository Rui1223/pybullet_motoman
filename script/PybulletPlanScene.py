#!/usr/bin/env python
from __future__ import division

import pybullet as p
import pybullet_data

import time
import sys
import os
import copy
import math
import numpy as np

import utils
from MotomanRobot import MotomanRobot
from WorkspaceTable import WorkspaceTable
from Planner import Planner

import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from pybullet_motoman.msg import EEPoses
from pybullet_motoman.msg import ObjectPoseBox

from pybullet_motoman.srv import MotionPlanning, MotionPlanningResponse
from pybullet_motoman.srv import SingleJointChange, SingleJointChangeResponse
from pybullet_motoman.srv import ExecuteTrajectory, ExecuteTrajectoryRequest
from pybullet_motoman.msg import EdgeConfigs
from pybullet_motoman.srv import AddVirtualBox, AddVirtualBoxResponse

### This class defines a PybulletPlanScene class which
### sets up the robot, table in the planning scene (known geometries)
### updates the object geometry based on perception
### updates robot configuration based on the information from the simulated execution scene
### performs planning for a query (from current configuration to targe configuration)

VERBOSE = False

class PybulletPlanScene(object):

    def __init__(self, args):

        ### read in relevant ros parameters for scene generation
        basePosition, baseOrientation, urdfFile, object_mesh_path, \
        leftArmHomeConfiguration, rightArmHomeConfiguration, \
        standingBase_dim, table_dim, table_offset_x = self.readROSParam()
        rospack = rospkg.RosPack() ### https://wiki.ros.org/Packages
        self.rosPackagePath = rospack.get_path("pybullet_motoman")

        ### set the server for the pybullet planning scene
        # self.planningClientID = p.connect(p.DIRECT)
        self.planningClientID = p.connect(p.GUI)

        ### create a planner assistant
        self.planner_p = Planner(
            self.rosPackagePath, self.planningClientID,
            isObjectInLeftHand=False, isObjectInRightHand=False,
            objectInLeftHand=None, objectInRightHand=None)

        ### configure the robot
        self.configureMotomanRobot(urdfFile, basePosition, baseOrientation, \
                leftArmHomeConfiguration, rightArmHomeConfiguration, False)
        ### set up the workspace
        self.setupWorkspace(standingBase_dim, table_dim, table_offset_x, object_mesh_path, False)


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
        print("leftArmHomeConfiguration")
        print(leftArmHomeConfiguration)

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

        # while not rospy.has_param('/workspace_table/transitCenterHeight'):
        #     rospy.sleep(0.2)
        # transitCenterHeight = rospy.get_param('/workspace_table/transitCenterHeight')

        return basePosition, baseOrientation, urdfFile, object_mesh_path, \
            leftArmHomeConfiguration, rightArmHomeConfiguration, \
            standingBase_dim, table_dim, table_offset_x


    def rosInit(self):
        ### This function specifies the role of a node instance for this class ###
        ### and initialize a ros node ###
        ### specify the role of a node instance for this class
        ### claim the service
        motion_planning_server = rospy.Service("motion_planning", MotionPlanning, self.motion_plan_callback)
        single_joint_change_server = rospy.Service(
                                "single_joint_change", SingleJointChange, self.single_joint_change_callback)
        add_virtual_box_server = rospy.Service(
                "add_virtual_box", AddVirtualBox, self.add_virtual_box_callback)
        rospy.init_node("pybullet_plan_scene", anonymous=True)


    def updateEEPoseInPlanScene(self):
        ### get the information on whether the object is in hand (which object? which hand?)
        ### by looking at the topic "ee_poses"
        start_time = time.time()
        ee_poses_msg = rospy.wait_for_message("ee_poses", EEPoses)
        isObjectInLeftHand = ee_poses_msg.isObjectInLeftHand
        isObjectInRightHand = ee_poses_msg.isObjectInRightHand
        self.planner_p.updateManipulationStatus(
                isObjectInLeftHand, isObjectInRightHand, self.workspace_p.object_geometries, self.robot_p)
        # print("+++++++++++++++++++PRINT OBJECT INFO++++++++++++++++++++")
        # print(self.workspace_p.object_geometries)
        if VERBOSE:
            print('updateEEPoseInPlanScene takes time: %f' % (time.time()-start_time))

    def updateObjectPoseInPlanScene(self, object_pose):
        start_time = time.time()

        self.workspace_p.updateObjectMesh(object_pose)
        if VERBOSE:
            print('updateObjectPoseInPlanScene takes time: %f' % (time.time()-start_time))


    def updateRobotConfigurationInPlanScene(self):
        ### get the current robot config from real scene by looking at the topic "joint_states"
        start_time = time.time()

        joint_states_msg = rospy.wait_for_message('left_right_joint_states', JointState)
        joint_values = list(joint_states_msg.position)
        self.robot_p.resetArmConfig(joint_values)
        self.robot_p.updateSingleArmConfig(joint_values[0:7], "Left")
        self.robot_p.updateSingleArmConfig(joint_values[7:14], "Right")
        self.robot_p.setRestPoses(
            self.robot_p.leftArmCurrConfiguration, self.robot_p.rightArmCurrConfiguration,
            self.robot_p.rightHandCurrConfiguration) ### the right hand config is what it is
        if VERBOSE:
            print('updateRobotConfigurationInPlanScene takes time: %f' % (time.time()-start_time))


    def updateInPlanSceneFromRealScene(self, object_pose):
        start_time = time.time()

        ### update the object in the plan scene based on real scene
        if object_pose.dims:
            ### only update the object pose
            ### if the object pose is updated from the last planning
            self.updateObjectPoseInPlanScene(object_pose)
            # print("object geometries: ")
            # print(self.workspace_p.object_geometries.keys()[0])
            # print("table known geometries: ")
            # print([self.workspace_p.known_geometries[1]])
            # isCollision = self.planner_p.collisionAgent_p.collisionCheck_object_knownGEO([self.workspace_p.object_geometries.keys()[0]], [self.workspace_p.known_geometries[1]])
            # print("is there a collision? ", isCollision)
            # time.sleep(1000000)
        ### update the robot in the plan scene based on real scene
        self.updateRobotConfigurationInPlanScene()
        ### update the information to see (there is an object in any of the hand)
        self.updateEEPoseInPlanScene()
        if VERBOSE:
            print('updateInPlanSceneFromRealScene takes time: %f' % (time.time()-start_time))


    def transit_motion_planning(self, req):
        start_time = time.time()
        armType = req.armType
        motionType = req.motionType
        ### synchronize with the real scene so as to get the object and robot initial pose
        self.updateInPlanSceneFromRealScene(req.object_pose)
        # time.sleep(1000000)
        if VERBOSE:
            print('transit_motion_planning after updateInPlanSceneFromRealScene takes time: %f' % (time.time()-start_time))

        if armType == "Left":
            initialPose = copy.deepcopy(self.robot_p.left_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.leftArmCurrConfiguration)
            theme = "LeftTransit"
        else:
            initialPose = copy.deepcopy(self.robot_p.right_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.rightArmCurrConfiguration)
            theme = "RightTransit"

        targetPose = [[req.gripper_pose.position.x, req.gripper_pose.position.y,
            req.gripper_pose.position.z], [req.gripper_pose.orientation.x,
            req.gripper_pose.orientation.y, req.gripper_pose.orientation.z,
            req.gripper_pose.orientation.w]]
        if armType == "Left":
            ### check the targetPose
            isPoseValid, configToGraspPose = self.planner_p.generateConfigBasedOnPose(
                        targetPose, self.robot_p, self.workspace_p, armType, motionType)
            # print("configToGraspPose")
            # print(configToGraspPose)
        else:
            ###### HARD-CODED PART #########
            ### for right hand, we make a conservative target pose
            print("ARE YOU TRYING TO GENERATE A CONSERVATIVE TARGET GRASP?????????????")
            isPoseValid, configToGraspPose = self.planner_p.generateConfigBasedOnPose(
                        targetPose, self.robot_p, self.workspace_p, armType, motionType)
            # isPoseValid, GraspPose, configToGraspPose = self.planner_p.generateConservativeTargetGrasp(
            #             targetPose, self.robot_p, self.workspace_p, armType, motionType)
            print('YES, I TRY IT HARD, MAN')
            # print("Let's set this as the home configuration")
            print("configToGraspPose")
            print(configToGraspPose)
            print("++++++++++++++++++++++++++++++++++++++")
            print("++++++++++++++++++++++++++++++++++++++")

            # if armType == "Right":
            #     time.sleep(1000000)

        if VERBOSE:
            print('transit_motion_planning after generateConfigBasedOnPose takes time: %f' % (time.time()-start_time))


        if not isPoseValid:
            print("this pose is not even valid, let alone motion planning")
            return False
        else:
            print("congrats on the target pose, now pre-grasp pose, please")
            # if armType == "Right":
            #     time.sleep(1000000)
            ### reset rest pose
            if armType == "Left":
                self.robot_p.setRestPoses(configToGraspPose, self.robot_p.rightArmCurrConfiguration, self.robot_p.rightHandCurrConfiguration)
            else:
                self.robot_p.setRestPoses(self.robot_p.leftArmCurrConfiguration, configToGraspPose, self.robot_p.rightHandCurrConfiguration)
                # time.sleep(100000)
            ### given the targetPose, get the preGraspPose, which is 10cm behind along z axis
            isPoseValid, preGraspPose, configToPreGraspPose = self.planner_p.generatePreGrasp(
                                targetPose, self.robot_p, self.workspace_p, armType, motionType)
            if VERBOSE:
                print('transit_motion_planning after generatePreGrasp takes time: %f' % (time.time()-start_time))

            if not isPoseValid:
                print("the pre-grasp pose is not valid, thus the grasp pose is deemed as invalid as well")
                return False

        print("both grasp pose and pre-grasp pose are legitimate")
            ## first check if we can direct connect current pose to pre-grasp_pose
        isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
            initialConfig, configToPreGraspPose, self.robot_p, self.workspace_p, armType, motionType)
        if VERBOSE:
            print('transit_motion_planning after checkEdgeValidity_DirectConfigPath takes time: %f' % (time.time()-start_time))

        if isDirectPathValid:
            ### it is feasible to directly move from current pose to pre-grasp pose
            print("the poses can be directly connected")
            ### now we need to generate a trajectory for executor to execute
            config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                    initialConfig, configToPreGraspPose, min_degree=math.pi / 90)
            result_traj = []
            result_traj.append(config_edge_traj)
        else:
            ### if it's not possible, then we have to trigger motion planning
            result_traj = self.planner_p.AstarPathFinding(
                    initialConfig, configToPreGraspPose,
                    self.robot_p, self.workspace_p, armType, motionType)
            # result_traj = self.planner_p.shortestPathPlanning(
            #         initialConfig, configToPreGraspPose, theme,
            #         self.robot_p, self.workspace_p, armType, motionType)
        if VERBOSE:
            print('transit_motion_planning before execution takes time: %f' % (time.time()-start_time))

        ## the planning has been finished, either success or failure
        if result_traj != []:
            print("the transit path for %s arm is successfully found" % armType)
            ### now we need to call a service call to execute the path in the execution scene
            execute_success = self.serviceCall_execute_trajectory(
                                result_traj, armType, self.robot_p.motomanRJointNames)
            if VERBOSE:
                print('transit_motion_planning after execution takes time: %f' % (time.time()-start_time))

        else:
            print("the transit path for %s is not successfully found" % armType)
            return False 
        
        ### you are reaching here since pre-grasp pose has been reached
        ### just do a translation to reach the final grasp pose (10cm in the positive z axis)
        config_edge_traj = self.planner_p.generateTrajectory_approachToGrasp(
                                                preGraspPose, self.workspace_p, armType, self.robot_p)
        if VERBOSE:
            print('transit_motion_planning after generateTrajectory_approachToGrasp takes time: %f' % (time.time()-start_time))

        print("+++++++++++++++++++++++++++")
        print("finish approachToGrasp traj for arm: %s" % (armType))        
        result_traj = []
        result_traj.append(config_edge_traj)
        execute_success = self.serviceCall_execute_trajectory(
                            result_traj, armType, self.robot_p.motomanRJointNames)
        if VERBOSE:
            print('transit_motion_planning takes time: %f' % (time.time()-start_time))

        print("the execution has been finished\n")

        # time.sleep(1000000)



        return True


    def transfer_motion_planning(self, req):
        start_time = time.time()
        armType = req.armType
        motionType = req.motionType
        ### synchronize with the real scene so as to get the object and robot initial pose
        self.updateInPlanSceneFromRealScene(req.object_pose)
        if VERBOSE:
            print('transfer_motion_planning after updateInPlanSceneFromRealScene takes time: %f' % (time.time()-start_time))

        if armType == "Left":
            initialPose = copy.deepcopy(self.robot_p.left_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.leftArmCurrConfiguration)
            theme = "LeftTransfer"
        else:
            initialPose = copy.deepcopy(self.robot_p.right_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.rightArmCurrConfiguration)
            theme = "RightTransfer"

        ### check if there is a collision between the object and the table (id: 1)
        isCollision = self.planner_p.collisionAgent_p.collisionCheck_object_knownGEO(
                                        self.workspace_p.object_geometries, [self.workspace_p.known_geometries[1]], dist_threshold=0.05)
        if isCollision:
            ### lift it up
            targetPose = [[initialPose[0][0], initialPose[0][1],
                            initialPose[0][2]+0.06], initialPose[1]]

            isPoseValid, configToTargetPose = self.planner_p.generateConfigBasedOnPose(
                                targetPose, self.robot_p, self.workspace_p, armType, motionType)
            if not isPoseValid:
                print("this pose is not even valid, let alone motion planning")
                return False
            else:
                print("the grasp pose is valid, proceed to planning")

            # isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
            #     initialConfig, configToTargetPose, self.robot_p, self.workspace_p, armType, motionType)

            # if isDirectPathValid:
            #     config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
            #                                                     initialConfig, configToTargetPose)
            #     result_traj = []
            #     result_traj.append(config_edge_traj)
            # else:
            #     result_traj = self.planner_p.AstarPathFinding(
            #             initialConfig, configToTargetPose,
            #             self.robot_p, self.workspace_p, armType, motionType)

            config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                            initialConfig, configToTargetPose, min_degree=math.pi / 90 / 2)
            result_traj = []
            result_traj.append(config_edge_traj)
            
            if result_traj != []:
                execute_success = self.serviceCall_execute_trajectory(
                                            result_traj, armType, self.robot_p.motomanRJointNames)
            print("lift up %s arm finished" % armType)
            ### now depends on left or right arm
            ### we want to move the arm along side (either left or right)
            initialConfig = configToTargetPose


        targetPose = [[req.gripper_pose.position.x, req.gripper_pose.position.y,
            req.gripper_pose.position.z], [req.gripper_pose.orientation.x,
            req.gripper_pose.orientation.y, req.gripper_pose.orientation.z,
            req.gripper_pose.orientation.w]]
        ### check if the target pose is valid
        isPoseValid, configToGraspPose = self.planner_p.generateConfigBasedOnPose(
                    targetPose, self.robot_p, self.workspace_p, armType, motionType)

        if VERBOSE:
            print('transfer_motion_planning after generateConfigBasedOnPose takes time: %f' % (time.time()-start_time))

        if not isPoseValid:
            print("this pose is not even valid, let alone motion planning")
            return False
        else:
            print("the grasp pose is valid, proceed to planning")

        ### first check if we can direct connect current pose to grasp_pose
        isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
            initialConfig, configToGraspPose, self.robot_p, self.workspace_p, armType, motionType)
        # isDirectPathValid = False
        # isDirectPathValid, config_edge_traj = self.planner_p.checkEdgeValidity_cartesianMove(
        #                     configToGraspPose, initialPose, targetPose, self.robot_p, self.workspace_p, armType, motionType)
        # print("========================is DirectPathValid: ", isDirectPathValid)
        # time.sleep(1000000)
        if VERBOSE:
            print('transfer_motion_planning after checkEdgeValidity_DirectConfigPath takes time: %f' % (time.time()-start_time))
        # isDirectPathValid = False
        if isDirectPathValid:
            ### it is feasible to directly move from current pose to grasp pose
            print("the poses can be directly connected")
            ### now we need to generate a trajectory for executor to execute
            if armType == "Left":
                config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                            initialConfig, configToGraspPose, min_degree=math.pi / 90)
            else:
                config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                            initialConfig, configToGraspPose)               
            result_traj = []
            result_traj.append(config_edge_traj)
        else:
            # ### if it's not possible, then we have to trigger motion planning
            result_traj = self.planner_p.AstarPathFinding(
                    initialConfig, configToGraspPose,
                    self.robot_p, self.workspace_p, armType, motionType)
            # result_traj = self.planner_p.shortestPathPlanning(
            #         initialConfig, configToGraspPose, theme, 
            #         self.robot_p, self.workspace_p, armType, motionType)
        if VERBOSE:
            print('transfer_motion_planning before excution takes time: %f' % (time.time()-start_time))
        ## the planning has been finished, either success or failure
        if result_traj != []:
            print("the tranfer path for %s arm is successfully found" % armType)
            ### now we need to call a service call to execute the path in the execution scene
            execute_success = self.serviceCall_execute_trajectory(
                                    result_traj, armType, self.robot_p.motomanRJointNames)
            print("the execution has been finished")
            print("let me the know where the object is")
            print(self.workspace_p.object_geometries)
            if VERBOSE:
                print('transfer_motion_planning takes time: %f' % (time.time()-start_time))

            return True
        else:
            print("the transfer path for %s is not successfully found" % armType)
            if VERBOSE:
                print('transfer_motion_planning takes time: %f' % (time.time()-start_time))

            return False


    def moveAway_motion_planning(self, req):

        armType = req.armType
        motionType = req.motionType
        ### synchronize with the real scene so as to get the object and robot initial pose
        self.updateInPlanSceneFromRealScene(req.object_pose)
        if armType == "Left":
            initialPose = copy.deepcopy(self.robot_p.left_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.leftArmCurrConfiguration)
            # theme = "LeftMoveAway"
            theme = "LeftTransit"

        else:
            initialPose = copy.deepcopy(self.robot_p.right_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.rightArmCurrConfiguration)
            theme = "RightTransit"

        ### lift it up
        targetPose = [[initialPose[0][0], initialPose[0][1],
                        initialPose[0][2]+0.04], initialPose[1]]

        isPoseValid, configToTargetPose = self.planner_p.generateConfigBasedOnPose(
                            targetPose, self.robot_p, self.workspace_p, armType, motionType)
        if not isPoseValid:
            print("this pose is not even valid, let alone motion planning")
            return False
        else:
            print("the grasp pose is valid, proceed to planning")

        isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
            initialConfig, configToTargetPose, self.robot_p, self.workspace_p, armType, motionType)

        if isDirectPathValid:
            config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                            initialConfig, configToTargetPose)
            result_traj = []
            result_traj.append(config_edge_traj)
        else:
            result_traj = self.planner_p.AstarPathFinding(
                    initialConfig, configToTargetPose,
                    self.robot_p, self.workspace_p, armType, motionType)
        
        if result_traj != []:
            execute_success = self.serviceCall_execute_trajectory(
                                        result_traj, armType, self.robot_p.motomanRJointNames)
        print("lift up %s arm finished" % armType)

        ### now depends on left or right arm
        ### we want to move the arm along side (either left or right)
        currConfig = configToTargetPose
        if armType == "Left":
            ### for the left arm, move the left
            targetPose = [[targetPose[0][0], targetPose[0][1]+0.4,
                            targetPose[0][2]], targetPose[1]]
        else:
            ### for the right arm, move to right
            targetPose = [[targetPose[0][0], targetPose[0][1]-0.4,
                            targetPose[0][2]], targetPose[1]]
        isPoseValid, configToTargetPose = self.planner_p.generateConfigBasedOnPose(
                            targetPose, self.robot_p, self.workspace_p, armType, motionType)

        # print("Let's set this as the home configuration")
        # print("configToTargetPose")
        # print(configToTargetPose)
        # print("++++++++++++++++++++++++++++++++++++++")
        # print("++++++++++++++++++++++++++++++++++++++")

        if not isPoseValid:
            print("this pose is not even valid, let alone motion planning")
            return False
        else:
            print("the grasp pose is valid, proceed to planning")
        
        isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
            currConfig, configToTargetPose, self.robot_p, self.workspace_p, armType, motionType)
        if isDirectPathValid:
            config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                            currConfig, configToTargetPose)
            result_traj = []
            result_traj.append(config_edge_traj)
        else:
            result_traj = self.planner_p.AstarPathFinding(
                    currConfig, configToTargetPose,
                    self.robot_p, self.workspace_p, armType, motionType)

        execute_success = self.serviceCall_execute_trajectory(
                                    result_traj, armType, self.robot_p.motomanRJointNames)
        print("move alongside %s arm finished" % armType)
        # # ** reset to home pose
        # # home_pose = [0.0, 0.0, 1.57, -1.57, 0.0, 0.0, 0.0]
        # home_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
        #     configToTargetPose, home_pose, self.robot_p, self.workspace_p, armType, motionType)

        # if isDirectPathValid:
        #     config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
        #                                                     configToTargetPose, home_pose)
        #     result_traj = []
        #     result_traj.append(config_edge_traj)
        # else:
        #     result_traj = self.planner_p.AstarPathFinding(
        #             configToTargetPose, home_pose,
        #             self.robot_p, self.workspace_p, armType, motionType)
        
        # if result_traj != []:
        #     execute_success = self.serviceCall_execute_trajectory(
        #                                 result_traj, armType, self.robot_p.motomanRJointNames)

        return True


    def moveAway3_motion_planning(self, req):

        armType = req.armType
        motionType = req.motionType
        ### synchronize with the real scene so as to get the object and robot initial pose
        self.updateInPlanSceneFromRealScene(req.object_pose)
        if armType == "Left":
            initialPose = copy.deepcopy(self.robot_p.left_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.leftArmCurrConfiguration)
            # theme = "LeftMoveAway"
            theme = "LeftTransit"

        else:
            initialPose = copy.deepcopy(self.robot_p.right_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.rightArmCurrConfiguration)
            theme = "RightTransit"
        if armType == 'Left':

            ### lift it up
            targetPose = [[initialPose[0][0], initialPose[0][1],
                            initialPose[0][2]+0.04], initialPose[1]]

            isPoseValid, configToTargetPose = self.planner_p.generateConfigBasedOnPose(
                                targetPose, self.robot_p, self.workspace_p, armType, motionType)
            if not isPoseValid:
                print("this pose is not even valid, let alone motion planning")
                return False
            else:
                print("the grasp pose is valid, proceed to planning")

            isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
                initialConfig, configToTargetPose, self.robot_p, self.workspace_p, armType, motionType)

            if isDirectPathValid:
                config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                                initialConfig, configToTargetPose)
                result_traj = []
                result_traj.append(config_edge_traj)
            else:
                result_traj = self.planner_p.AstarPathFinding(
                        initialConfig, configToTargetPose,
                        self.robot_p, self.workspace_p, armType, motionType)
            
            if result_traj != []:
                execute_success = self.serviceCall_execute_trajectory(
                                            result_traj, armType, self.robot_p.motomanRJointNames)
            print("lift up %s arm finished" % armType)

            ### now depends on left or right arm
            ### we want to move the arm along side (either left or right)
            currConfig = configToTargetPose
            if armType == "Left":
                ### for the left arm, move the left
                targetPose = [[targetPose[0][0], targetPose[0][1]+0.4,
                                targetPose[0][2]], targetPose[1]]
            else:
                ### for the right arm, move to right
                targetPose = [[targetPose[0][0], targetPose[0][1]-0.4,
                                targetPose[0][2]], targetPose[1]]
            isPoseValid, configToTargetPose = self.planner_p.generateConfigBasedOnPose(
                                targetPose, self.robot_p, self.workspace_p, armType, motionType)

            # print("Let's set this as the home configuration")
            # print("configToTargetPose")
            # print(configToTargetPose)
            # print("++++++++++++++++++++++++++++++++++++++")
            # print("++++++++++++++++++++++++++++++++++++++")

            if not isPoseValid:
                print("this pose is not even valid, let alone motion planning")
                return False
            else:
                print("the grasp pose is valid, proceed to planning")
            
            isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
                currConfig, configToTargetPose, self.robot_p, self.workspace_p, armType, motionType)
            if isDirectPathValid:
                config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                                currConfig, configToTargetPose)
                result_traj = []
                result_traj.append(config_edge_traj)
            else:
                result_traj = self.planner_p.AstarPathFinding(
                        currConfig, configToTargetPose,
                        self.robot_p, self.workspace_p, armType, motionType)

            execute_success = self.serviceCall_execute_trajectory(
                                        result_traj, armType, self.robot_p.motomanRJointNames)
            print("move alongside %s arm finished" % armType)
            # # ** reset to home pose
            # # home_pose = [0.0, 0.0, 1.57, -1.57, 0.0, 0.0, 0.0]
            # home_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
            #     configToTargetPose, home_pose, self.robot_p, self.workspace_p, armType, motionType)

            # if isDirectPathValid:
            #     config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
            #                                                     configToTargetPose, home_pose)
            #     result_traj = []
            #     result_traj.append(config_edge_traj)
            # else:
            #     result_traj = self.planner_p.AstarPathFinding(
            #             configToTargetPose, home_pose,
            #             self.robot_p, self.workspace_p, armType, motionType)
            
            # if result_traj != []:
            #     execute_success = self.serviceCall_execute_trajectory(
            #                                 result_traj, armType, self.robot_p.motomanRJointNames)
            return True

        if armType == 'Right':
            #### (not used) configToTargetPose = [-3.051709074651789, -0.7437459017506597, -0.11702544303157998, 1.2418171296172877, 1.5097524020385686, -1.6338341900612185, 0.4236664846169708]
        #     configToTargetPose = [-3.1282064342345585, -0.8423079221535875, -0.5623394341156668, 1.0641399123596846, 1.420238382384334, -1.3375485840697974, -0.83051516392]
        # else:
        #     configToTargetPose = [2.1838748733518063, -0.9209540227290458, 0.8832032430221721, 1.1669805304419556, 2.2862132885171125, -1.7084464602195035, -2.920715879126091]
            print("first lift it up")

        ###################################################################################################################
        targetPose = [[initialPose[0][0], initialPose[0][1],
                        initialPose[0][2]+0.0], initialPose[1]]
        isPoseValid, configToTargetPose = self.planner_p.generateConfigBasedOnPose(
                            targetPose, self.robot_p, self.workspace_p, armType, motionType)
        if not isPoseValid:
            print("this pose is not even valid, let alone motion planning")
            return False
        else:
            print("the grasp pose is valid, proceed to planning")

        isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
            initialConfig, configToTargetPose, self.robot_p, self.workspace_p, armType, motionType)

        if isDirectPathValid:
            config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                            initialConfig, configToTargetPose, min_degree=math.pi / 90)
            result_traj = []
            result_traj.append(config_edge_traj)
        else:
            result_traj = self.planner_p.AstarPathFinding(
                    initialConfig, configToTargetPose,
                    self.robot_p, self.workspace_p, armType, motionType)
        
        if result_traj != []:
            execute_success = self.serviceCall_execute_trajectory(
                                        result_traj, armType, self.robot_p.motomanRJointNames)
        print("lift up %s arm finished" % armType)
        ###################################################################################################################

        print("then move back to home position")
        initialConfig = copy.deepcopy(configToTargetPose)
        configToTargetPose = [-3.1282064342345585, -0.8423079221535875, -0.5623394341156668, 1.0641399123596846, 1.420238382384334, -1.3375485840697974, -0.83051516392]

        isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
            initialConfig, configToTargetPose, self.robot_p, self.workspace_p, armType, motionType)

        if isDirectPathValid:
            config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                            initialConfig, configToTargetPose, min_degree=math.pi / 90)
            result_traj = []
            result_traj.append(config_edge_traj)
        else:
            result_traj = self.planner_p.AstarPathFinding(
                    initialConfig, configToTargetPose,
                    self.robot_p, self.workspace_p, armType, motionType)
        
        if result_traj != []:
            execute_success = self.serviceCall_execute_trajectory(
                                        result_traj, armType, self.robot_p.motomanRJointNames)
        print("lift up %s arm finished" % armType)

        return True


    def moveAway1_motion_planning(self, req):

        armType = req.armType
        motionType = req.motionType
        ### synchronize with the real scene so as to get the object and robot initial pose
        self.updateInPlanSceneFromRealScene(req.object_pose)
        if armType == "Left":
            initialPose = copy.deepcopy(self.robot_p.left_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.leftArmCurrConfiguration)
            # theme = "LeftMoveAway"
            theme = "LeftTransit"

        else:
            initialPose = copy.deepcopy(self.robot_p.right_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.rightArmCurrConfiguration)
            theme = "RightTransit"


        if armType == "Left":
            ### for the left arm, move the left
            targetPose = [[initialPose[0][0], initialPose[0][1]+0.1,
                            initialPose[0][2]], initialPose[1]]
        else:
            ### for the right arm, move to right
            targetPose = [[initialPose[0][0], initialPose[0][1]-0.1,
                            initialPose[0][2]], initialPose[1]]
        isPoseValid, configToTargetPose = self.planner_p.generateConfigBasedOnPose(
                            targetPose, self.robot_p, self.workspace_p, armType, motionType)

        if not isPoseValid:
            print("this pose is not even valid, let alone motion planning")
            return False
        else:
            print("the grasp pose is valid, proceed to planning")
        
        isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
            initialConfig, configToTargetPose, self.robot_p, self.workspace_p, armType, motionType)
        if isDirectPathValid:
            config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                            initialConfig, configToTargetPose)
            result_traj = []
            result_traj.append(config_edge_traj)
        else:
            result_traj = self.planner_p.AstarPathFinding(
                    initialConfig, configToTargetPose,
                    self.robot_p, self.workspace_p, armType, motionType)

        execute_success = self.serviceCall_execute_trajectory(
                                    result_traj, armType, self.robot_p.motomanRJointNames)
        print("move alongside %s arm finished" % armType)

        initialConfig = copy.deepcopy(configToTargetPose)
        initialPose = copy.deepcopy(targetPose)

        ### lift it up
        targetPose = [[initialPose[0][0], initialPose[0][1],
                        initialPose[0][2]+0.25], initialPose[1]]

        isPoseValid, configToTargetPose = self.planner_p.generateConfigBasedOnPose(
                            targetPose, self.robot_p, self.workspace_p, armType, motionType)
        if not isPoseValid:
            print("this pose is not even valid, let alone motion planning")
            return False
        else:
            print("the grasp pose is valid, proceed to planning")

        isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
            initialConfig, configToTargetPose, self.robot_p, self.workspace_p, armType, motionType)

        if isDirectPathValid:
            config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                            initialConfig, configToTargetPose)
            result_traj = []
            result_traj.append(config_edge_traj)
        else:
            result_traj = self.planner_p.AstarPathFinding(
                    initialConfig, configToTargetPose,
                    self.robot_p, self.workspace_p, armType, motionType)
        
        if result_traj != []:
            execute_success = self.serviceCall_execute_trajectory(
                                        result_traj, armType, self.robot_p.motomanRJointNames)
        print("lift up %s arm finished" % armType)

        # ** reset to home pose
        # home_pose = [0.0, 0.0, 1.57, -1.57, 0.0, 0.0, 0.0]
        # home_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
        #     configToTargetPose, home_pose, self.robot_p, self.workspace_p, armType, motionType)

        # if isDirectPathValid:
        #     config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
        #                                                     configToTargetPose, home_pose)
        #     result_traj = []
        #     result_traj.append(config_edge_traj)
        # else:
        #     result_traj = self.planner_p.AstarPathFinding(
        #             configToTargetPose, home_pose,
        #             self.robot_p, self.workspace_p, armType, motionType)
        
        # if result_traj != []:
        #     execute_success = self.serviceCall_execute_trajectory(
        #                                 result_traj, armType, self.robot_p.motomanRJointNames)

        return True


    def moveAway2_motion_planning(self, req):

        armType = req.armType
        motionType = req.motionType
        ### synchronize with the real scene so as to get the object and robot initial pose
        self.updateInPlanSceneFromRealScene(req.object_pose)
        if armType == "Left":
            initialPose = copy.deepcopy(self.robot_p.left_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.leftArmCurrConfiguration)
            # theme = "LeftMoveAway"
            theme = "LeftTransit"

        else:
            initialPose = copy.deepcopy(self.robot_p.right_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.rightArmCurrConfiguration)
            theme = "RightTransit"


        if armType == "Left":
            ### for the left arm, move the left
            targetPose = [[initialPose[0][0], initialPose[0][1]+0.1,
                            initialPose[0][2]], initialPose[1]]
        else:
            ### for the right arm, move to right
            targetPose = [[initialPose[0][0], initialPose[0][1]-0.1,
                            initialPose[0][2]], initialPose[1]]
        isPoseValid, configToTargetPose = self.planner_p.generateConfigBasedOnPose(
                            targetPose, self.robot_p, self.workspace_p, armType, motionType)

        if not isPoseValid:
            print("this pose is not even valid, let alone motion planning")
            return False
        else:
            print("the grasp pose is valid, proceed to planning")
        
        isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
            initialConfig, configToTargetPose, self.robot_p, self.workspace_p, armType, motionType)
        if isDirectPathValid:
            config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                            initialConfig, configToTargetPose, min_degree=math.pi / 90)
            result_traj = []
            result_traj.append(config_edge_traj)
        else:
            result_traj = self.planner_p.AstarPathFinding(
                    initialConfig, configToTargetPose,
                    self.robot_p, self.workspace_p, armType, motionType)

        execute_success = self.serviceCall_execute_trajectory(
                                    result_traj, armType, self.robot_p.motomanRJointNames)
        print("move alongside %s arm finished" % armType)

        initialConfig = copy.deepcopy(configToTargetPose)
        initialPose = copy.deepcopy(targetPose)

        # ### lift it up
        # targetPose = [[initialPose[0][0], initialPose[0][1],
        #                 initialPose[0][2]+0.25], initialPose[1]]

        # isPoseValid, configToTargetPose = self.planner_p.generateConfigBasedOnPose(
        #                     targetPose, self.robot_p, self.workspace_p, armType, motionType)

        isPoseValid = True
        # configToTargetPose = [-3.051709074651789, -0.7437459017506597, -0.11702544303157998, 1.2418171296172877, 1.5097524020385686, -1.6338341900612185, 0.4236664846169708]
        configToTargetPose = [-3.1282064342345585, -0.8423079221535875, -0.5623394341156668, 1.0641399123596846, 1.420238382384334, -1.3375485840697974, -0.83051516392]

        if not isPoseValid:
            print("this pose is not even valid, let alone motion planning")
            return False
        else:
            print("the grasp pose is valid, proceed to planning")

        isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
            initialConfig, configToTargetPose, self.robot_p, self.workspace_p, armType, motionType)

        if isDirectPathValid:
            config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                            initialConfig, configToTargetPose, min_degree=math.pi / 90)
            result_traj = []
            result_traj.append(config_edge_traj)
        else:
            result_traj = self.planner_p.AstarPathFinding(
                    initialConfig, configToTargetPose,
                    self.robot_p, self.workspace_p, armType, motionType)
        
        if result_traj != []:
            execute_success = self.serviceCall_execute_trajectory(
                                        result_traj, armType, self.robot_p.motomanRJointNames)
        print("lift up %s arm finished" % armType)
        return True


    def reset_motion_planning(self, req):

        armType = req.armType
        motionType = req.motionType
        ### synchronize with the real scene so as to get the object and robot initial pose
        self.updateInPlanSceneFromRealScene(req.object_pose)
        if armType == "Left":
            initialPose = copy.deepcopy(self.robot_p.left_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.leftArmCurrConfiguration)
            theme = "LeftReset"
        else:
            initialPose = copy.deepcopy(self.robot_p.right_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.rightArmCurrConfiguration)
            theme = "RightReset"

        ### No matter left arm or right arm, the first step is always lift up (0.9cm)
        ### so as to leave the object (be safe)
        targetPose = [[initialPose[0][0], initialPose[0][1], initialPose[0][2] + 0.05], initialPose[1]]

        isPoseValid, configToTargetPose = self.planner_p.generateConfigBasedOnPose(
                            targetPose, self.robot_p, self.workspace_p, armType, motionType)
        if not isPoseValid:
            print("this pose is not even valid, let alone motion planning")
            return False
        else:
            print("the grasp pose is valid, proceed to planning")
        config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                        initialConfig, configToTargetPose)
        result_traj = []
        result_traj.append(config_edge_traj)
        execute_success = self.serviceCall_execute_trajectory(
                                    result_traj, armType, self.robot_p.motomanRJointNames)

        ### now depends on left or arm,

        ### we want to move the arm along side (either left or right)
        currConfig = configToTargetPose ### update the current configuration
        if armType == "Left":
            ### for the left hand, move to left
            targetPose = [[targetPose[0][0], targetPose[0][1]+0.4,
                            targetPose[0][2]], targetPose[1]]
        else:
            ### for the right hand, move to right
            targetPose = [[targetPose[0][0], targetPose[0][1]-0.4,
                            targetPose[0][2]], targetPose[1]]
        isPoseValid, configToTargetPose = self.planner_p.generateConfigBasedOnPose(
                            targetPose, self.robot_p, self.workspace_p, armType, motionType)
        print("=================================")
        print("=================================")
        print("=================================")
        print("reset config")
        print(configToTargetPose)
        if not isPoseValid:
            print("this pose is not even valid, let alone motion planning")
            return False
        else:
            print("the grasp pose is valid, proceed to planning")
        config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                        currConfig, configToTargetPose)
        result_traj = []
        result_traj.append(config_edge_traj)
        execute_success = self.serviceCall_execute_trajectory(
                                    result_traj, armType, self.robot_p.motomanRJointNames)

        print("reset %s arm finished\n" % armType)
        return True


    def approachToPlacement_motion_planning(self, req):

        start_time = time.time()
        armType = req.armType
        motionType = req.motionType
        ### synchronize with the real scene so as to get the object and robot initial pose
        self.updateInPlanSceneFromRealScene(req.object_pose)
        if VERBOSE:
            print('approachToPlacement_motion_planning after updateInPlanSceneFromRealScene takes time: %f' % (time.time()-start_time))

        if armType == "Left":
            initialPose = copy.deepcopy(self.robot_p.left_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.leftArmCurrConfiguration)
            theme = "LeftToPlace"
        else:
            initialPose = copy.deepcopy(self.robot_p.right_ee_pose)
            initialConfig = copy.deepcopy(self.robot_p.rightArmCurrConfiguration)
            theme = "RightToPlace"

        ### figure out the shortest distance between the table and the object
        distance_from_tableSurface = self.workspace_p.detectHeight()
        print("distance_from_tableSurface: ", distance_from_tableSurface)
        if distance_from_tableSurface > 0.02:
            distance_to_moveDown = distance_from_tableSurface - 0.02
            targetPose = [[initialPose[0][0], initialPose[0][1], \
                        initialPose[0][2] - distance_to_moveDown], initialPose[1]]
            isPoseValid, configToTargetPose = self.planner_p.generateConfigBasedOnPose(
                                targetPose, self.robot_p, self.workspace_p, armType, motionType)
            if VERBOSE:
                print('approachToPlacement_motion_planning after generateConfigBasedOnPose takes time: %f' % (time.time()-start_time))
            if not isPoseValid:
                print("this pose is not even valid, let alone motion planning")
                return False
            else:
                print("the grasp pose is valid, proceed to planning")
            config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                                                            initialConfig, configToTargetPose)
            if VERBOSE:
                print('approachToPlacement_motion_planning after DirectConfigPath takes time: %f' % (time.time()-start_time))        
            result_traj = []
            result_traj.append(config_edge_traj)
            execute_success = self.serviceCall_execute_trajectory(
                                        result_traj, armType, self.robot_p.motomanRJointNames)
            print("%s arm place object finished\n" % armType)
        else:
            print("the height is close enough, no need to approach")
        if VERBOSE:
            print('approachToPlacement_motion_planning takes time: %f' % (time.time()-start_time))

        return True

    def add_virtual_box_callback(self, req):
        start_time = time.time()
        half_extents = np.array(req.dims) / 2
        test_visual = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=half_extents,
                                          rgbaColor=[0, 0, 0, 1])
        _ = p.createMultiBody(baseVisualShapeIndex=test_visual, basePosition=req.position,
                              baseOrientation=req.orientation)
        res = AddVirtualBoxResponse()
        res.success = True
        print('add_virtual_box_cb take time: %f' % (time.time()-start_time))
        return res

    def motion_plan_callback(self, req):
        if req.motionType == "transit":
            isSuccess = self.transit_motion_planning(req)
            print("isSuccess: ", isSuccess)
            return MotionPlanningResponse(isSuccess)

        elif req.motionType == "transfer":
            isSuccess = self.transfer_motion_planning(req)
            print("isSuccess: ", isSuccess)
            return MotionPlanningResponse(isSuccess)

        elif req.motionType == "moveAway":
            # isSuccess = self.moveAway_motion_planning(req)  # up, then right

            # isSuccess = self.moveAway3_motion_planning(req)  # for handoff
            isSuccess = self.moveAway2_motion_planning(req)  # right hand direct grasp
            print("isSuccess: ", isSuccess)
            return MotionPlanningResponse(isSuccess)

        elif req.motionType == "reset":
            isSuccess = self.reset_motion_planning(req)
            print("isSuccess: ", isSuccess)
            return MotionPlanningResponse(isSuccess)

        elif req.motionType == "approachToPlacement":
            isSuccess = self.approachToPlacement_motion_planning(req)
            print("isSuccess: ", isSuccess)
            return MotionPlanningResponse(isSuccess)

        else:
            print("could not handle this type of motion")
            return MotionPlanningResponse(False)


    def single_joint_change_callback(self, req):
        ### update the robot config
        start_time = time.time()

        self.updateRobotConfigurationInPlanScene()
        armType = req.armType
        if armType == "Left":
            currArmConfig = copy.deepcopy(self.robot_p.leftArmCurrConfiguration)
        else:
            currArmConfig = copy.deepcopy(self.robot_p.rightArmCurrConfiguration)

        targetArmConfig = copy.deepcopy(currArmConfig)
        ### given the joint_name, figure out the index of the joint
        joint_index = self.robot_p.motomanRJointNames.index(req.joint_name)
        rotate_radian = req.rotate_angle * math.pi / 180
        if armType == "Left":
            target_radian = targetArmConfig[joint_index] + rotate_radian
        else:
            target_radian = targetArmConfig[joint_index-7] + rotate_radian
        while target_radian > np.pi:
            target_radian -= np.pi*2
        while target_radian < -np.pi:
            target_radian += np.pi*2
        if armType == "Left":
            ### check if the given joint value exceeds the limit
            if (target_radian < self.robot_p.ll[joint_index]) or (target_radian > self.robot_p.ul[joint_index]):
                rospy.logerr("The joint value input for %s exceeds its limits" % req.joint_name)
                rospy.logerr(
                    "%f does not in the range [%f, %f]" % \
                    (target_radian, self.robot_p.ll[joint_index], self.robot_p.ul[joint_index]))
                return SingleJointChangeResponse(False)
            ### otherwise continue
            targetArmConfig[joint_index] = target_radian
        else:
            ### check if the given joint value exceeds the limit
            if (target_radian < self.robot_p.ll[joint_index]) or \
                        (target_radian > self.robot_p.ul[joint_index]):
                rospy.logerr("The joint value input for %s exceeds its limits" % req.joint_name)
                rospy.logerr(
                    "%f does not in the range [%f, %f]" % \
                    (target_radian, self.robot_p.ll[joint_index], self.robot_p.ul[joint_index]))
                return SingleJointChangeResponse(False)
            ### otherwise continue
            targetArmConfig[joint_index-7] = target_radian
            if VERBOSE:
                print('single_joint_change_cb before generateTrajectory_DirectConfigPath takes time: %f' % (time.time() - start_time))

        config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath_faster(
                        currArmConfig, targetArmConfig)
        if VERBOSE:
            print('single_joint_change_cb after generateTrajectory_DirectConfigPath takes time: %f' % (time.time() - start_time))

        result_traj = []
        result_traj.append(config_edge_traj)

        execute_success = self.serviceCall_execute_trajectory(
                                    result_traj, armType, self.robot_p.motomanRJointNames)
        print("change the %s of %s arm finished" % (req.joint_name, armType))
        if VERBOSE:
            print('single_joint_change_cb takes time: %f' % (time.time() - start_time))
        return SingleJointChangeResponse(True)


    def serviceCall_execute_trajectory(self, result_traj, armType, motomanRJointNames):
        ### here the result_traj has the format:
        ### [[edge1_configs], [edge2_configs], ...]
        start_time = time.time()
        rospy.wait_for_service("execute_trajectory")
        request = ExecuteTrajectoryRequest()
        request.armType = armType

        if armType == "Left":
            first_joint_index = 0
        else:
            first_joint_index = 7

        for edge_configs in result_traj:
            temp_edge_configs = EdgeConfigs()
            for config in edge_configs:
                joint_state = JointState()
                joint_state.name = motomanRJointNames[first_joint_index:first_joint_index+7]
                joint_state.position = config
                temp_edge_configs.edge_configs.append(joint_state)
            request.trajectory.append(temp_edge_configs)

        try:
            executeIt = rospy.ServiceProxy("execute_trajectory", ExecuteTrajectory)
            print('before sending request to execute_trajectory, time: %f' % (time.time() - start_time))
            start_time = time.time()
            success = executeIt(request.trajectory, request.armType)
            print('execute_trajectory, time: %f' % (time.time() - start_time))
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
            object_mesh_path, isPhysicsTurnOn):
        ### The function sets up the workspace
        self.workspace_p = WorkspaceTable(self.robot_p.basePosition,
            standingBase_dim, table_dim, table_offset_x,
            os.path.join(self.rosPackagePath, object_mesh_path),
            isPhysicsTurnOn, self.planningClientID)


def main(args):
    pybullet_plan_scene = PybulletPlanScene(args)
    pybullet_plan_scene.planner_p.loadSamples()
    pybullet_plan_scene.rosInit()
    rate = rospy.Rate(10) ### 10hz

    rospy.spin()



if __name__ == '__main__':
    main(sys.argv)


############################# below is not used but kept for legacy #############################
    # def transit_motion_planning_bk(self, req):

    #     armType = req.armType
    #     motionType = req.motionType
    #     ### synchronize with the real scene so as to get the object and robot initial pose
    #     self.updateInPlanSceneFromRealScene(req.object_pose)

    #     if armType == "Left":
    #         initialPose = copy.deepcopy(self.robot_p.left_ee_pose)
    #         initialConfig = copy.deepcopy(self.robot_p.leftArmCurrConfiguration)
    #         theme = "LeftTransit"
    #     else:
    #         initialPose = copy.deepcopy(self.robot_p.right_ee_pose)
    #         initialConfig = copy.deepcopy(self.robot_p.rightArmCurrConfiguration)
    #         theme = "RightTransit"

    #     targetPose = [[req.gripper_pose.position.x, req.gripper_pose.position.y,
    #         req.gripper_pose.position.z], [req.gripper_pose.orientation.x,
    #         req.gripper_pose.orientation.y, req.gripper_pose.orientation.z,
    #         req.gripper_pose.orientation.w]]
    #     ### check if the target pose is valid
    #     isPoseValid, configToGraspPose = self.planner_p.generateConfigBasedOnPose(
    #                 targetPose, self.robot_p, self.workspace_p, armType, motionType)
    #     if not isPoseValid:
    #         print("this pose is not even valid, let alone motion planning")
    #         return False
    #     else:
    #         print("the grasp pose is valid, generate pre-grasp")
    #         # print("target pose:")
    #         # print(targetPose)
    #         # print("configToGraspPose: ")
    #         # print(configToGraspPose)
    #         ### update rest pose
    #         self.robot_p.setRestPoses(
    #             self.robot_p.leftArmCurrConfiguration, self.robot_p.rightArmCurrConfiguration, self.robot_p.rightHandCurrConfiguration)
    #         isPoseValid, preGraspPose, configToPreGraspPose = self.planner_p.generatePreGrasp(
    #                             targetPose, self.robot_p, self.workspace_p, armType, motionType)
    #         if not isPoseValid:
    #             print("the pre-grasp pose is not valid, thus the grasp pose is deemed as invalid as well")
    #             return False
    #     print("both grasp pose and pre-grasp pose are legitimate")
    #     # print("preGraspPose:")
    #     # print(preGraspPose)
    #     # print("configToPreGraspPose: ")
    #     # print(configToPreGraspPose)
    #     print("proceed to planning")

    #     ## first check if we can direct connect current pose to pre-grasp_pose
    #     isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
    #         initialConfig, configToPreGraspPose, self.robot_p, self.workspace_p, armType, motionType)
    #     if isDirectPathValid:
    #         ### it is feasible to directly move from current pose to pre-grasp pose
    #         print("the poses can be directly connected")
    #         ### now we need to generate a trajectory for executor to execute
    #         config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
    #                                                 initialConfig, configToPreGraspPose)
    #         result_traj = []
    #         result_traj.append(config_edge_traj)
    #     else:
    #         ### if it's not possible, then we have to trigger motion planning
    #         result_traj = self.planner_p.shortestPathPlanning(
    #                 initialConfig, configToPreGraspPose, theme,
    #                 self.robot_p, self.workspace_p, armType, motionType)

    #     ## the planning has been finished, either success or failure
    #     if result_traj != []:
    #         print("the transit path for %s arm is successfully found" % armType)
    #         ### now we need to call a service call to execute the path in the execution scene
    #         execute_success = self.serviceCall_execute_trajectory(
    #                             result_traj, armType, self.robot_p.motomanRJointNames)
    #     else:
    #         print("the transit path for %s arm is not successfully found" % armType)
    #         return False

    #     ### you are reaching here since pre-grasp pose has been reached
    #     ### just do a translation to reach the final grasp pose
    #     # config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
    #     #                                         configToPreGraspPose, configToGraspPose)
    #     config_edge_traj = self.planner_p.generateTrajectory_cartesianMove(
    #                                             preGraspPose, targetPose, armType, robot)
    #     result_traj = []
    #     result_traj.append(config_edge_traj)

    #     execute_success = self.serviceCall_execute_trajectory(
    #                         result_traj, armType, self.robot_p.motomanRJointNames)
    #     print("the execution has been finished\n")
    #     return True
