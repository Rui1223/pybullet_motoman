#!/usr/bin/env python
from __future__ import division

import pybullet as p
import pybullet_data

import threading
import numpy as np
import os
import shutil
import time
import cv2

import rospkg
from PIL import Image
from skimage.io import imread, imsave

### This file defines the camera for the purpose of getting the images of the simulated scene ###

class SimulatedCamera(object):

    def __init__(self, tablePosition, table_dim, 
        camera_extrinsic, camera_intrinsic,
        scene_index, saveImages,
        server):
        ### get the server
        self.server = server
        self.scene_index = scene_index
        self.camera_extrinsic = np.array(camera_extrinsic)
        self.viewMatrix = p.computeViewMatrix(
                cameraEyePosition=[self.camera_extrinsic[0][3], self.camera_extrinsic[1][3], self.camera_extrinsic[2][3]],
                cameraTargetPosition=[tablePosition[0]+table_dim[0]/2+0.3, tablePosition[1], tablePosition[2]+table_dim[2]/2],
                cameraUpVector=[-self.camera_extrinsic[0][1], -self.camera_extrinsic[1][1], -self.camera_extrinsic[2][1]])
        self.projectionMatrix = p.computeProjectionMatrixFOV(
                fov=camera_intrinsic[0],
                aspect=camera_intrinsic[1],
                nearVal=camera_intrinsic[2],
                farVal=camera_intrinsic[3])

        if saveImages:
            self.createImageDataFolder()


    def takeRGBImage(self):

        ### get the image
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
            width=1280,
            height=720,
            viewMatrix=self.viewMatrix,
            projectionMatrix=self.projectionMatrix,
            physicsClientId=self.server
        )

        return cv2.cvtColor(rgbImg, cv2.COLOR_RGB2BGR), depthImg


    def saveImage(imageType, Img, frame_idx):
        if imageType == 'rgb':
            ### rgb
            cv2.imwrite(self.rgbImg_path + "/frame%06d.png"%frame_idx, cv2.cvtColor(Img, cv2.COLOR_RGB2BGR))
        elif imageType == "depth":
            ### depth
            cv2.imwrite(self.depthImg_path + "/frame%06d.png"%frame_idx, (1000*Img).astype(np.uint16))
        else:
            ### segmentation
            np.save(self.segmentationImg_path + "/frame%06d"%frame_idx, Img)


    def createImageDataFolder(self):
        ### create a folder to store all the images generated from the current scene
        ### input -> scene_index
        ### output -> rgbImg_path, depthImg_path, segmentationImg_path, data_path (self member)
        # self.img_path = "/home/rui/Documents/research/motoman_ws/src/pybullet_motoman/src/sensor_images/" + self.scene_index
        rospack = rospkg.RosPack()
        self.img_path = os.path.join(rospack.get_path("pybullet_motoman"), "src/sensor_images", self.scene_index)
        self.rgbImg_path = self.img_path + "/rgb"
        self.depthImg_path = self.img_path + "/depth"
        self.segmentationImg_path = self.img_path + "/segmentation"
        self.data_path = self.img_path + "/data"

        if os.path.exists(self.img_path):
            shutil.rmtree(self.img_path)
        os.makedirs(self.img_path)

        if os.path.exists(self.rgbImg_path):
            shutil.rmtree(self.rgbImg_path)
        os.makedirs(self.rgbImg_path)

        if os.path.exists(self.depthImg_path):
            shutil.rmtree(self.depthImg_path)
        os.makedirs(self.depthImg_path)

        if os.path.exists(self.segmentationImg_path):
            shutil.rmtree(self.segmentationImg_path)
        os.makedirs(self.segmentationImg_path)

        if os.path.exists(self.data_path):
            shutil.rmtree(self.data_path)
        os.makedirs(self.data_path)
