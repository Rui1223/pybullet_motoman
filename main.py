from __future__ import division
import pybullet as p
import pybullet_data

import sys
import os
import time

from Experiment import Experiment

### indicate the id of the current scene you are working on
scene_index = sys.argv[1]
### specify the experiment mode
### mode "e" : large-experiment mode, where you want to turn both planning server and executing server as invisible
### mode "v": visualization mode, where you want to see the final execution in the true scene
### mode "p": planning mode, where you want to visualize the planning process for purposes like debugging
exp_mode = sys.argv[2]
saveImages = (sys.argv[3] in ('y', 'Y')) ### decide whether to save images or not

EXP = Experiment(scene_index, exp_mode, saveImages)
EXP.runExperiment()

time.sleep(10000)


