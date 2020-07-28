from __future__ import division
import pybullet as p
import pybullet_data

import threading
import numpy as np
import os
import shutil
import time

from PIL import Image
from skimage.io import imread, imsave

class AzureKineticCamera(object):
    def __init__(self, scene_index, tablePosition, table_dim):
        self.scene_index = scene_index
        self.rgbImg_path, self.depthImg_path, self.segmentationImg_path, self.data_path = self.createImageFolder(scene_index)
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
                nearVal=0.4,
                farVal=3.47)


    def takeImage(self, clientId, saveImages, frame_idx):

        ### get the image
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
        width=1280,
        height=720,
        viewMatrix=self.viewMatrix,
        projectionMatrix=self.projectionMatrix,
        physicsClientId=clientId)

        ### save the images ###
        if saveImages:
            ### rgb
            imsave(self.rgbImg_path + "/rgb_" + str(frame_idx) + ".png", rgbImg)
            ### depth
            imsave(self.depthImg_path + "/depth_" + str(frame_idx) + ".png", depthImg)
            ### segmentation
            imsave(self.segmentationImg_path + "/segment_" + str(frame_idx) + ".png", segImg)



    def createImageFolder(self, scene_index):
        ### create a folder to store all the images generated from the current scene
        img_path = os.getcwd() + "/sensor_images/" + scene_index
        rgbImg_path = img_path + "/rgb"
        depthImg_path = img_path + "/depth"
        segmentationImg_path = img_path + "/segmentation"
        data_path = img_path + "/data"

        if os.path.exists(img_path):
            shutil.rmtree(img_path)
        os.mkdir(img_path) 

        if os.path.exists(rgbImg_path):
            shutil.rmtree(rgbImg_path)
        os.mkdir(rgbImg_path)

        if os.path.exists(depthImg_path):
            shutil.rmtree(depthImg_path)
        os.mkdir(depthImg_path)

        if os.path.exists(segmentationImg_path):
            shutil.rmtree(segmentationImg_path)
        os.mkdir(segmentationImg_path)

        if os.path.exists(data_path):
            shutil.rmtree(data_path)
        os.mkdir(data_path)        

        return rgbImg_path, depthImg_path, segmentationImg_path, data_path
