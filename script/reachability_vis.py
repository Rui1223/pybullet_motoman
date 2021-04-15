"""
For each of the query position of gripper, generate reachability map
"""
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy

from PybulletExecutionScene import PybulletExecutionScene
from PybulletPlanScene import PybulletPlanScene

robot = None
workspace = None

def main():
    # sample point in the table, and test if the arm can move there
    # also sample in the z axis
    global planner
    global robot, workspace
    # rospy.init_node("reachability_test", anonymous=True)

    # pybullet_execution_scene = PybulletExecutionScene(["", "3", "n", "003_cracker_box"])

    pybullet_plan_scene = PybulletPlanScene(["", "3", "n", "003_cracker_box"])
    pybullet_plan_scene.planner_p.loadSamples()
    pybullet_plan_scene.rosInit()


    planner = pybullet_plan_scene.planner_p
    robot = pybullet_plan_scene.robot_p
    table_map_dim = pybullet_plan_scene.workspace_p.table_dim
    workspace = pybullet_plan_scene.workspace_p
    x_range = [workspace.tablePosition[0]-workspace.table_dim[0]/2, workspace.tablePosition[0]]#+workspace.table_dim[0]/2]
    y_range = [workspace.tablePosition[1]-workspace.table_dim[1]/2, workspace.tablePosition[1]+workspace.table_dim[1]/2]
    z_offset = 0.1
    z_range = [workspace.tablePosition[2]+workspace.table_dim[2]/2+0.1, workspace.tablePosition[2]+workspace.table_dim[2]/2+0.01+0.1]

    num = 20
    z_num = 1
    num_complex = complex(0, num)
    z_num_complex = complex(0, z_num)
    # x_range = np.linspace(x_range[0], x_range[1], num)
    # y_range = np.linspace(y_range[0], y_range[1], num)
    # z_range = np.linspace(z_range[0], z_range[1], num)

    valid_grasp_pos = np.load("valid_pose_right_vertical_%f.npy" % (z_offset))
    for i in range(len(valid_grasp_pos)):
        pos = valid_grasp_pos[i,:3].tolist()
        spot_v = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.01, rgbaColor=[0,1,0,1], physicsClientId=robot.server)
        spot_m = p.createMultiBody(baseVisualShapeIndex=spot_v, basePosition=pos, physicsClientId=robot.server)

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # scat = ax.scatter(X, Y, Z, c=grasp_from_right_vertical.flatten())
    # fig.colorbar(scat, shrink=0.5, aspect=5)

    # plt.show()

    # fig.savefig('grasp_from_right_vertical.png')
    # reset arm
    robot.setSingleArmToConfig([0.,0.,0.,0.,0.,0.,0], 'Right')
    rospy.sleep(10000000000.)
main()