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
def ik_single_query(pos, ori, armType='Right'):

    if armType == "Left":
        ee_idx = robot.left_ee_idx
        first_joint_index = 0
        db_start_idx = 0
    else:
        ee_idx = robot.right_ee_idx
        first_joint_index = 7
        db_start_idx = 12

    # temp_rot_matrix = p.getMatrixFromQuaternion(grasp_pose[1])
    # ### local z-axis of the end effector
    # temp_approaching_direction = [temp_rot_matrix[2], temp_rot_matrix[5], temp_rot_matrix[8]]
    # temp_pos = list(np.array(grasp_pose[0]) - 0.025*np.array(temp_approaching_direction))
    preGrasp_pose = [pos, ori]
    ### check the IK the pre-grasp pose
    ori_q_preGraspIK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO, 
                            endEffectorLinkIndex=ee_idx, 
                            targetPosition=preGrasp_pose[0], 
                            targetOrientation=preGrasp_pose[1], 
                            lowerLimits=robot.ll, upperLimits=robot.ul, 
                            jointRanges=robot.jr, restPoses=robot.rp,
                            maxNumIterations=10000, residualThreshold=0.001,
                            physicsClientId=robot.server)
    singleArmConfig_IK = list(ori_q_preGraspIK[first_joint_index:first_joint_index+7])
    singleArmConfig_IK, isIKValid = planner.AdjustIKBasedOnJointLimit(singleArmConfig_IK, robot, armType)
    isPoseValid = False
    if isIKValid:
        isPoseValid = planner.checkPoseIK(singleArmConfig_IK, ee_idx, preGrasp_pose, robot, workspace, armType, motionType='transit')
    # if not isPoseValid:
    #     print('Pybullet failed. Linear IK')
    #     singleArmConfig_IK = planner.linear_inverse_kinematics(armType, preGrasp_pose, robot.rp, robot, start_pose=None)
    #     isIKValid = False
    #     isPoseValid = False
    #     if singleArmConfig_IK is not None:
    #         singleArmConfig_IK, isIKValid = planner.AdjustIKBasedOnJointLimit(singleArmConfig_IK, robot, armType)
    #         if isIKValid:
    #             isPoseValid = planner.checkPoseIK(
    #                     singleArmConfig_IK, ee_idx, preGrasp_pose, robot, workspace, armType, motionType)

    print('result: %d' % (isPoseValid))
    return isPoseValid

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
    z_offset = 0.12
    z_range = [workspace.tablePosition[2]+workspace.table_dim[2]/2+z_offset, workspace.tablePosition[2]+workspace.table_dim[2]/2+0.01+z_offset]

    num = 40
    z_num = 1
    num_complex = complex(0, num)
    z_num_complex = complex(0, z_num)
    # x_range = np.linspace(x_range[0], x_range[1], num)
    # y_range = np.linspace(y_range[0], y_range[1], num)
    # z_range = np.linspace(z_range[0], z_range[1], num)

    X, Y, Z = np.mgrid[x_range[0]:x_range[1]:num_complex, y_range[0]:y_range[1]:num_complex, z_range[0]:z_range[1]:z_num_complex]
    print('X shape: ')
    print(X.shape)

    rp = planner.useRestPoseFromJsonFile(robot, 'Right', 12)
    robot.setRestPoses(robot.leftArmCurrConfiguration, rp[7:14], robot.rightHandCurrConfiguration)
    # generate map for grasping pose from right
    grasp_from_right_vertical = np.zeros(X.shape)
    valid_grasp_pos = []


    # ori = [0., 0.707, 0.707, 0.]
    ori = [1., 0., 0., 0.]

    angles = list(p.getEulerFromQuaternion(ori))
    angles = np.array(angles)
    ori_range = 0. / 180 * np.pi
    noise = np.random.random(3) * ori_range - ori_range / 2.
    angles = angles + noise

    ori = p.getQuaternionFromEuler(angles)
    ori = list(ori)
    for i in range(num):
        for j in range(num):
            for k in range(z_num):
                print('i, j, k: %d, %d, %d' % (i, j, k))
                pos = [X[i,j,k], Y[i,j,k], Z[i,j,k]]
                print('orientation: ')
                print(ori)
                print('noise: ')
                print(noise)
                if ik_single_query(pos, ori):
                    grasp_from_right_vertical[i,j,k] = 1
                    valid_grasp_pos.append(pos + ori)
                    spot_v = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.01, rgbaColor=[0,1,0,1], physicsClientId=robot.server)
                    spot_m = p.createMultiBody(baseVisualShapeIndex=spot_v, basePosition=pos, physicsClientId=robot.server)

                else:
                    grasp_from_right_vertical[i,j,k] = 0

    valid_grasp_pos = np.array(valid_grasp_pos)
    np.save("valid_pose_right_vertical_%f.npy" % (z_offset), valid_grasp_pos)
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