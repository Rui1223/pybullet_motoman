from __future__ import division
import pybullet as p
import pybullet_data

class CollisionChecker(object):
    def __init__(self, server):
        self.server = server

    def collisionCheck_selfCollision(self, robotGEO):
        isCollision = False
        contacts = p.getContactPoints(bodyA=robotGEO, bodyB=robotGEO, physicsClientId=self.server)
        if len(contacts) != 0:
            # print("self collision!")
            isCollision = True

        return isCollision


    def collisionCheck_knownGEO(self, robotGEO, knownGEO):
        isCollision = False
        ### loop through all known geometries in the workspace
        for g in knownGEO:
            contacts = p.getContactPoints(robotGEO, g, physicsClientId=self.server)
            if len(contacts) != 0:
                isCollision = True
                # print("collision with known GEO")
                break

        return isCollision
