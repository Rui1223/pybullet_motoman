from __future__ import division
import pybullet as p
import pybullet_data

import threading
import numpy as np
import os
import shutil
import time
import cv2

from PIL import Image
from skimage.io import imread, imsave

class AzureKineticCamera(object):

    def __init__(self, tablePosition, table_dim, scene_index, saveImages):
        self.scene_index = scene_index
        self.cameraFinish = False
        self.camera_extrinsic = np.array(
            [[-0.0182505, -0.724286,  0.689259, 0.329174],
             [-0.999453,  0.0322427,  0.00741728,  -0.036492],
             [-0.0275958, -0.688746, -0.724478, 1.24839],
             [0.0, 0.0, 0.0, 1.0]])
        self.viewMatrix = p.computeViewMatrix(
                cameraEyePosition=[self.camera_extrinsic[0][3], self.camera_extrinsic[1][3], self.camera_extrinsic[2][3]],
                cameraTargetPosition=[tablePosition[0]+table_dim[0]/2+0.3, tablePosition[1], tablePosition[2]+table_dim[2]/2],
                cameraUpVector=[-self.camera_extrinsic[0][1], -self.camera_extrinsic[1][1], -self.camera_extrinsic[2][1]])
        self.projectionMatrix = p.computeProjectionMatrixFOV(
                fov=90.0,
                aspect=1.78,
                nearVal=0.1,
                farVal=1.5)

        if saveImages:
            self.createImageDataFolder()


    def takeImage(self, clientId, saveImages, frame_idx):

        ### get the image
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
            width=1280,
            height=720,
            viewMatrix=self.viewMatrix,
            projectionMatrix=self.projectionMatrix,
            physicsClientId=clientId
        )

        ### save the images ###
        if saveImages:
            ### rgb
            cv2.imwrite(self.rgbImg_path + "/frame%06d.png"%frame_idx, cv2.cvtColor(rgbImg, cv2.COLOR_RGB2BGR))
            ### depth
            cv2.imwrite(self.depthImg_path + "/frame%06d.png"%frame_idx, (1000*depthImg).astype(np.uint16))
            ### segmentation
            np.save(self.segmentationImg_path + "/frame%06d"%frame_idx, segImg)

        return cv2.cvtColor(rgbImg, cv2.COLOR_RGB2BGR)


    def createImageDataFolder(self):
        ### create a folder to store all the images generated from the current scene
        ### input -> scene_index
        ### output -> rgbImg_path, depthImg_path, segmentationImg_path, data_path (self member)
        # self.img_path = "/home/rui/Documents/research/motoman_ws/src/pybullet_motoman/src/sensor_images/" + self.scene_index
        self.img_path = os.path.join("sensor_images", self.scene_index)
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
