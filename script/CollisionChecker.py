from __future__ import division
import pybullet as p
import pybullet_data
import IPython

class CollisionChecker(object):
    def __init__(self, server):
        self.server = server

    def collisionCheck_selfCollision(self, robotGEO):
        isCollision = False
        contacts = p.getContactPoints(bodyA=robotGEO, bodyB=robotGEO, physicsClientId=self.server)
        if len(contacts) == 0:
            pass
        else:
            # print("self collision!")
            ### check each contact
            for contact in contacts:
                # print("link-to-link collision: ")
                # print(str(contact[3]) + ": " + str(contact[4]))
                if (contact[3] == 23 and contact[4] == 25) or (contact[3] == 28 and contact[4] == 30) \
                    or (contact[3] == 24 and contact[4] == 25) or (contact[3] == 29 and contact[4] == 30):
                    ### we can allow collisions among the links of the robotiq hand 
                    ### (mounted on the right arm)
                    pass
                else:
                    isCollision = True
                    break

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
