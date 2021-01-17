from __future__ import division
import pybullet as p
import pybullet_data

import time
import math
import IPython

import utils
from CollisionChecker import CollisionChecker

class Executor(object):

    def __init__(self, server):
        self.executingServer = server
    

    def executePath(self, path, robot, armType):
        ### path is a list of configs
        for i in range(len(path)-1):
            current_state = path[i]
            next_state = path[i+1]
            self.state_transition(current_state, next_state, robot, armType)


    def state_transition(self, n1, n2, robot, armType):

        # nseg = 25
        min_degree = math.pi / 90
        nseg = int(max(
            abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]), abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6])) / min_degree)
        if nseg == 0: nseg += 1
        print("nseg: " + str(nseg))

        for i in range(1, nseg+1):
            interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
            interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
            interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
            interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
            interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
            interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
            interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
            intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]

            robot.moveSingleArm(intermNode, armType)
            # curr_waypoint = utils.convertRobotConfig_singleArm(intermNode, robot, armType, self.executingServer)
            # self.traj.append(curr_waypoint)
            # if (self.isObjectInLeftHand and armType == "Left") or (self.isObjectInRightHand and armType == "Right"):
            #     self.updateRealObjectBasedonLocalPose(robot, armType)

            p.stepSimulation(physicsClientId=self.executingServer)
            time.sleep(0.05)

