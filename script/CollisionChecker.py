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
                    or (contact[3] == 24 and contact[4] == 25) or (contact[3] == 29 and contact[4] == 30) \
                    or (contact[3] == 6 and contact[4] == 9) or (contact[3] == 19 and contact[4] == 16) \
                    or (contact[3] == 22 and contact[4] == 25):
                    ### we can allow collisions among the links of the robotiq hand 
                    ### (mounted on the right arm)
                    ### we also allow collision between the gripper and the wrist (both left and right hand)
                    pass
                else:
                    isCollision = True
                    break

        return isCollision


    def collisionCheck_robot_knownGEO(self, robotGEO, knownGEO, armType):
        isCollision = False
        ### loop through all known geometries in the workspace
        for g in knownGEO:
            contacts = p.getClosestPoints(robotGEO, g, distance=0., physicsClientId=self.server)
            if len(contacts) != 0:
                for contact in contacts:
                    print("body-to-body collision: ")
                    print(str(contact[1]) + ": " + str(contact[2]))
                    print("link-to-link collision: ")
                    print(str(contact[3]) + ": " + str(contact[4])) 
                isCollision = True
               
                # print("collision with known GEO")
                break

        return isCollision


    def collisionCheck_robot_objectGEO(self, 
                robotGEO, objectGEO, armType, isObjectInLeftHand, isObjectInRightHand):
        # print("+++++++++++++++++++")
        # print("isObjectInLeftHand: ", isObjectInLeftHand)
        # print("isObjectInRightHand: ", isObjectInRightHand)
        isCollision = False
        ### loop through all object geometries in the workspace
        for g in objectGEO:
            # print('server:')
            # print(self.server)
            # print('incollisioncheck:')
            # print(robotGEO)
            # print(g)
            contacts = p.getClosestPoints(robotGEO, g, distance=0., physicsClientId=self.server)
            # contacts = p.getContactPoints(robotGEO, g, physicsClientId=self.server)
            if len(contacts) != 0:
                for contact in contacts:
                    if contact[8] >= 0:
                        ### This is a fake collision (>=0: separation, <0: penetration)
                        continue
                    if (isObjectInLeftHand == True and contact[3] == 9):
                        continue
                    if (isObjectInRightHand == True and contact[3] == 19):
                        continue
                    if (isObjectInRightHand == True and contact[3] > 19):
                        continue
                    print("body-to-body collision: ")
                    print(str(contact[1]) + ": " + str(contact[2]))
                    print("link-to-link collision: ")
                    print(str(contact[3]) + ": " + str(contact[4]))
                    # print("contact position on robotGEO")
                    # print(str(contact[5]))
                    # print("contact position on objectGEO")
                    # print(str(contact[6]))
                    # print("contact distance:")
                    # print(str(contact[8]))
                    if (contact[3] == 10 and armType == "Left") or \
                            (contact[3] == 20 and armType == "Right"):
                        ### we allow the object to be slightly contact with the end effector
                        pass
                    elif (contact[3] == 9 and armType == "Left" and contact[8] >= 0) or \
                            (contact[3] == 19 and armType == "Right" and contact[8] >= 0):
                        ### we allow the object to be slightly contact with the hand
                        pass
                    else:
                        isCollision = True
                        # print("collision with object GEO")
                        break

        return isCollision


    def collisionCheck_object_knownGEO(self, objectGEO, knownGEO, dist_threshold=0.):
        isCollision = False
        ### loop through all objectGEO and knownGEO
        for object_g in objectGEO:
            for known_g in knownGEO:
                # contacts = p.getContactPoints(object_g, known_g, physicsClientId=self.server)
                contacts = p.getClosestPoints(object_g, known_g, distance=dist_threshold, physicsClientId=self.server)
                # print(object_g)
                # print(known_g)
                # print("contact")
                # print(contacts)
                if len(contacts) != 0:
                    for contact in contacts:
                        print("body-to-body collision: ")
                        print(str(contact[1]) + ": " + str(contact[2]))
                        print("link-to-link collision: ")
                        print(str(contact[3]) + ": " + str(contact[4]))

                    isCollision = True
                    # print("collision with known GEO")
                    break
            if isCollision: break

        return isCollision

