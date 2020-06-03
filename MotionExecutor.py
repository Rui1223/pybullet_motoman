from __future__ import division
import pybullet as p
import pybullet_data

import time

from CollisionChecker import CollisionChecker

class MotionExecutor(object):
    def __init__(self, server):
        self.executingServer = server

    def executePath(self, path, robot, handType, robotGEO):
        ### path is a list of configs
        for i in range(len(path)-1):
            current_state = path[i]
            next_state = path[i+1]
            self.local_move(current_state, next_state, robot, handType, robotGEO)


    def local_move(self, n1, n2, robot, handType, robotGEO):
        nseg = 25
        for i in xrange(0, nseg+1):
            interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
            interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
            interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
            interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
            interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
            interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
            interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
            intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
            robot.moveSingleArm(intermNode, robotGEO, handType, self.executingServer)

            time.sleep(0.05)

