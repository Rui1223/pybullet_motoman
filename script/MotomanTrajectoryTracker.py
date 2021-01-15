#! /usr/bin/env python
"""
Interface:
- /sda10f/sda10f_r1_controller/joint_trajectory_action
    type: control_msgs/FollowJointTrajectoryAction
    controller for left arm
- /sda10f/sda10f_r2_controller/joint_trajectory_action
    type: control_msgs/FollowJointTrajectoryAction
    controller for right arm
(above is hardware interface. We can simplify it by not dividing into left/right)

- joint_trajectory_action
type: control_msgs/FollowJointTrajectoryAction


Usage:
mimic the motoman driver, and provide action servers, so that user can use the same interface to control.

Design:
- Input: joint positions, velocities, time from start
- function:
    - sanity check: initial position and velocity
    - if valid, then forward the traj to lower-level controller (which directly obtain joint states from sim and track)
        - current implementation: wrap up traj in ROS topic
"""


import rospy

import actionlib

import control_msgs.msg
import trajectory_msgs.msg

from motoman_sim_controller.srv import PyBulletTrajTrack, PyBulletTrajTrackResponse
import numpy as np
class MotomanTrajectoryTracker(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self, name, pos_limit=None, vel_limit=[-2.95,2.95], acc_limit=[-10.,10.]):
        self._action_name = name
        self.pos_limit = pos_limit
        self.vel_limit = vel_limit
        self.acc_limit = acc_limit

        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        # calling service to the lower-level tracker (which directly talks to the simulator)
        self.sim_track_srv_name = 'pybullet_traj_track'
    
        self._as.start()

    def compute_vel_time(self, pos_traj, init_vel):
        # given configuration trajectory, compute the velocity and time
        # we assume linear velocity model, and don't have limit on time.
        # input: pos_traj: N*14, init_vel: 14
        vel_traj = [init_vel]
        time_traj = [0.]

        v0 = init_vel
        for i in range(1,len(pos_traj)):
            # we follow the following algo each iteration:
            # - initialize velocity by some fraction of max velocity
            # - compute duration t by t = 2(x1-x0)/(v0+v1)
            # - repeat the following:
            #   - compute v1 by: v1 = 2(x1-x0)/t-v0
            #   - compute acceleration by: (v1-v0)/t
            #   - check if v1 and acceleration are within bound. If not, then increase duration t. Otherwise break
            x0 = pos_traj[i-1]
            v0 = vel_traj[i-1]
            x1 = pos_traj[i]
            if x1[0] > x0[0]:
                    v10 = (-v0[0] + self.vel_limit[1]) / 2
            else:
                v10 = (-v0[0] + self.vel_limit[0]) / 2
            t = 2*(x1[0] - x0[0]) / (v0[0]+v10)
            while True:
                v1 = 2 * (x1-x0)/t - v0
                a = (v1-v0)/t
                # check for each dimension the constraint is satisfied
                if ((v1-self.vel_limit[1]>0) | (v1-self.vel_limit[0]<0)).sum()>0 or \
                   ((a-self.acc_limit[1]>0) | (a-self.acc_limit[0]<0)).sum()>0:
                    # if any item breaks the constraint, increase time
                    t = 2*t
                else:
                    break
            vel_traj.append(v1)
            time_traj.append(t+time_traj[-1])
        return vel_traj, time_traj
    
    def compute_acc(self, pos_traj, vel_traj, time_traj):
        # assume linear accleration
        acc_traj = []
        for i in range(len(pos_traj)-1):
            x1 = pos_traj[i]
            x2 = pos_traj[i+1]
            t1 = time_traj[i]
            t2 = time_traj[i+1]
            v1 = vel_traj[i]
            v2 = vel_traj[i+1]
            pos_dif = x2 - x1
            # if pos_dif > np.pi:
            #     pos_dif = pos_dif - 2*np.pi
            # if pos_dif < -np.pi:
            #     pos_dif = pos_dif + 2*np.pi
            a2 = -12*pos_dif/((t2-t1)**3) + 6*(v2+v1)/((t2-t1)**2)
            a1 = 6*pos_dif/((t2-t1)**2)-2*(v2+2*v1)/(t2-t1)
            acc_traj.append(a1)
        return acc_traj

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        rospy.loginfo("received goal, tracking...")
        # validate the trajectory: check initial state

        # send trajectory to lower-level controller that directly talks to simulator without lag
        traj = goal.trajectory  # trajectory_msgs/JointTrajectory

        print('traj: ')
        print(traj)
        # check if the velocity and time are specified or not. If not, then set them
        if len(traj.points[-1].velocities) == 0:  # -1 because we might have initial vel
            pos_traj = []
            for point in traj.points:
                pos_traj.append(point.positions)
            if len(traj.points[0].velocities) != 0:
                init_vel = traj.points[0].velocities
            else:
                init_vel = np.zeros(len(pos_traj[0]))
            pos_traj = np.array(pos_traj)
            init_vel = np.array(init_vel)
            vel_traj, time_traj = self.compute_vel_time(pos_traj, init_vel)
            # set the trajectory
            for i in range(len(traj.points)):
                traj.points[i].velocities = vel_traj[i]
                traj.points[i].time_from_start = rospy.Duration(time_traj[i])
            print('updated trajectory after adding velocity and time:')
            print(traj)
        print('waiting for service...')
        rospy.wait_for_service(self.sim_track_srv_name)
        print('service ready.')
        sim_track_srv = rospy.ServiceProxy(self.sim_track_srv_name, PyBulletTrajTrack)
        rospy.loginfo("calling service to track...")
        sim_track_response = sim_track_srv(traj)
        rospy.loginfo("tracking done.")
        # TODO: based on response, fill in feedback and result
        # currently only fill in blank values
        feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
        # feedback: shows incremental process. Now we just ignore it.
        # self._as.publish_feedback(feedback)

        result = control_msgs.msg.FollowJointTrajectoryResult()
        result.error_code = sim_track_response.error_code
        self._as.set_succeeded(result)
        # TODO: block calling when already working on one goal.          
        
if __name__ == '__main__':
    rospy.init_node('joint_trajectory_action')
    server = MotomanTrajectoryTracker(rospy.get_name())
    
    # unit testing
    pos_traj = np.random.random(size=(10,2)) * np.pi*2 - np.pi
    init_vel = np.zeros(2)
    vel_traj, time_traj = server.compute_vel_time(pos_traj, init_vel)
    print('----------unit testing for vel_time computer-----------')
    print('pos_traj:')
    print(pos_traj)
    print('vel_traj: ')
    print(vel_traj)
    print('time_traj:')
    print(time_traj)
    acc_traj = server.compute_acc(pos_traj, vel_traj, time_traj)
    print('acc_traj:')
    print(acc_traj)
    # validate to see if the velocity can get the position
    start_pos = pos_traj[0]
    computed_pos_traj = [start_pos]
    for i in range(len(vel_traj)-1):
        x1 = computed_pos_traj[i]
        v1 = vel_traj[i]
        v2 = vel_traj[i+1]
        t1 = time_traj[i]
        t2 = time_traj[i+1]
        x2 = (v1+v2)/2 * (t2-t1) + x1
        computed_pos_traj.append(x2)
    print('calculated pos_traj from vel:')
    print(computed_pos_traj)
    print('----------unit testing for vel_time computer-----------')
    rospy.spin()

