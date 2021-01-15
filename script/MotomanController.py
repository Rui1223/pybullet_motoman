#!/usr/bin/env python

from __future__ import print_function

from pybullet_motoman.srv import PyBulletTrajTrack, PyBulletTrajTrackResponse
import rospy
import pybullet as p
import numpy as np 
class MotomanController():
    def __init__(self, sim_step_func, time_step, robot_id, robot_joints, pybullet_client_id):
        # sim_step_func: the step function to run for each step during tracking
        self.s = rospy.Service('pybullet_traj_track', PyBulletTrajTrack, self.pybullet_traj_track)
        self.sim_step_func = sim_step_func
        self.client_id = pybullet_client_id
        self.robot_id = robot_id        
        self.time_step = time_step
        self.executing = False
        self.robot_joints = robot_joints
        # get joint names from robot, and map from joint_names -> jointIndex
        joint_name_to_id = {}
        num_joints = p.getNumJoints(self.robot_id, self.client_id)
        for i in range(num_joints):
            joint_info =p.getJointInfo(self.robot_id, i, self.client_id)
            joint_name_i = joint_info[1].decode('ascii')
            if joint_name_i in robot_joints:
                joint_name_to_id[joint_name_i] = i
        self.joint_name_to_id = joint_name_to_id

        rospy.loginfo("started MotomanController.")
    def compute_torque(self, x1, v1, t1, x2, v2, t2):
        pos_dif = x2 - x1
        # if pos_dif > np.pi:
        #     pos_dif = pos_dif - 2*np.pi
        # if pos_dif < -np.pi:
        #     pos_dif = pos_dif + 2*np.pi
        a2 = -12*pos_dif/((t2-t1)**3) + 6*(v2+v1)/((t2-t1)**2)
        a1 = 6*pos_dif/((t2-t1)**2)-2*(v2+2*v1)/(t2-t1)
        return a1
    def compute_pos_vel_acc_traj(self, traj):
        pos_traj = []
        vel_traj = []
        acc_traj = []
        joint_names = traj.joint_names
        points = traj.points
        for i in range(len(points)-1):
            joint_pos = []
            joint_vel = []
            joint_acc = []
            for name_idx in range(len(joint_names)):
                name = joint_names[name_idx]
                x1 = points[i].positions[name_idx]
                v1 = points[i].velocities[name_idx]
                t1 = points[i].time_from_start.to_sec()
                x2 = points[i+1].positions[name_idx]
                v2 = points[i+1].velocities[name_idx]
                t2 = points[i+1].time_from_start.to_sec()
                pos_dif = x2 - x1
                # if circular
                # if pos_dif > np.pi:
                #     pos_dif = pos_dif - 2*np.pi
                # if pos_dif < -np.pi:
                #     pos_dif = pos_dif + 2*np.pi
                ts = np.arange(t1, t2, self.time_step)

                a2 = -12*pos_dif/((t2-t1)**3) + 6*(v2+v1)/((t2-t1)**2)
                a1 = 6*pos_dif/((t2-t1)**2)-2*(v2+2*v1)/(t2-t1)
                # calcualte pos, vel, acc for each time
                pos = x1 +  v1 * (ts - t1) + a1 * ((ts-t1)**2)/2 + a2 * ((ts-t1)**3)/6
                vel = v1 + a1 * (ts - t1) + a2 * ((ts-t1)**2)/2
                acc = a1 + a2 * ts

                joint_pos.append(pos)
                joint_vel.append(vel)
                joint_acc.append(acc)
            joint_pos = np.array(joint_pos).T
            joint_vel = np.array(joint_vel).T
            joint_acc = np.array(joint_acc).T
            pos_traj.append(joint_pos)
            vel_traj.append(joint_vel)
            acc_traj.append(joint_acc)
        pos_traj = np.concatenate(pos_traj, 0)
        vel_traj = np.concatenate(vel_traj, 0)
        acc_traj = np.concatenate(acc_traj, 0)
        return pos_traj, vel_traj, acc_traj

    def pybullet_traj_track(self, req):
        self.executing = True
        traj = req.traj
        #** talk to pybullet to track the trajectory
        # (Synchornized Version) loop over waypoints and keep calling simulator.
        # - Each time get the state and time from sim
        # - using the time to find the next waypoint for tracking
        # - calculate the torque needed for current time
        # - apply torque to simulator for this step

        # Note: the given joint_names may not be the full joint names.
        #   in this case, we have to maintain the rest of joint to have velocity 0
        joint_names = traj.joint_names
        points = traj.points

        joint_name_to_id = self.joint_name_to_id
        pos_traj, vel_traj, acc_traj = self.compute_pos_vel_acc_traj(traj)
        nt = 0
        next_waypoint_id = 0
        rate = rospy.Rate(500) ### 10hz
        while True:
            rospy.loginfo('waypoint id: %d' % (next_waypoint_id))
            rospy.loginfo("time: %f" % (nt*self.time_step))
            # sense
            self.sim_step_func()
            
            # tracking
            # find the next waypoint time
            for i in range(next_waypoint_id, len(points)):
                if points[i].time_from_start.to_sec() > nt * self.time_step:
                    break
            if i == len(points):
                # end of tracking
                break
            next_waypoint_id = i
            # calculate control and apply torque
            cur_states = []
            cur_vels = []
            torques = []
            forces = []
            for name_idx in range(len(self.robot_joints)):
                name = self.robot_joints[name_idx]
                id = joint_name_to_id[name]
                # we have to loop over all free joints, since we need to calculate the force        
                state = p.getJointState(self.robot_id, id, self.client_id)
                x1 = state[0]
                v1 = state[1]

                cur_states.append(x1)
                cur_vels.append(v1)

                t1 = nt*self.time_step
                if name in joint_names:
                    # in specified trajectory
                    x2 = points[next_waypoint_id].positions[name_idx]
                    v2 = points[next_waypoint_id].velocities[name_idx]
                    t2 = points[next_waypoint_id].time_from_start.to_sec()
                    torque = self.compute_torque(x1, v1, t1, x2, v2, t2)
                else:
                    torque = 0.  # we want to make sure the other joints stay fixed
                torques.append(torque)
                #print('torque: ', torque)
                # apply control
                #p.setJointMotorControl2(self.robot_id, id, p.TORQUE_CONTROL, force=torque, physicsClientId=self.client_id)
                # p.setJointMotorControl2(self.robot_id, id, p.VELOCITY_CONTROL, force=0,physicsClientId=self.client_id)
                # p.setJointMotorControl2(self.robot_id, id, p.TORQUE_CONTROL, force=torque, physicsClientId=self.client_id)
            rospy.loginfo('current state: ')
            rospy.loginfo(cur_states)
            rospy.loginfo('calculated state:')
            rospy.loginfo(pos_traj[nt])
            rospy.loginfo('current velocity:')
            rospy.loginfo(cur_vels)
            rospy.loginfo('calculated velocity:')
            rospy.loginfo(vel_traj[nt])
            rospy.loginfo('current torque:')
            rospy.loginfo(torques)
            rospy.loginfo('calculated torque:')
            rospy.loginfo(acc_traj[nt])

            forces = p.calculateInverseDynamics(self.robot_id, cur_states, cur_vels, torques, self.client_id)

            for name_idx in range(len(self.robot_joints)):
                name = self.robot_joints[name_idx]
                if name not in joint_names:
                    # we only handle joints from traj
                    continue
                id = joint_name_to_id[name]
                p.setJointMotorControl2(self.robot_id, id, p.VELOCITY_CONTROL, force=0,physicsClientId=self.client_id)
                p.setJointMotorControl2(self.robot_id, id, p.TORQUE_CONTROL, force=forces[name_idx], physicsClientId=self.client_id)


            # step once
            p.stepSimulation(self.client_id)
            rate.sleep()
            # update step count
            nt += 1
            # if nt == 5:
            #     exit()

        # TODO: asynchronized version

        #** get the tracking response
        response = PyBulletTrajTrackResponse()
        response.error_code = response.SUCCESSFUL
        self.executing = False

        return response


if __name__ == "__main__":
    rospy.init_node('pybullet_traj_track')
    executor = pybulletExecutor()
    print("Ready to track trajectories")
    rospy.spin()
