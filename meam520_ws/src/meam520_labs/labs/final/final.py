
import sys
import numpy as np
from copy import deepcopy
from math import pi
import math
import pdb
import rospy
import sys
import scipy


sys.path.append("../")
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector
from lib.calculateFK import FK
from lib.IK_position_null import IK
from scipy.spatial.transform import Rotation as R
# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

def find_vertical_axis_and_adjust_yaw(transformation_matrix):
    # Extract the rotation matrix (upper 3x3 of the transformation matrix)
    rotation_matrix = transformation_matrix[:3, :3]
    # Vertical direction in the world frame, assuming block's up is world's down
    vertical_direction = np.array([0, 0, -1])
    # Calculate the dot product between each axis and the vertical direction
    cosines = np.dot(rotation_matrix.T, vertical_direction)
    # Find the axis that is most aligned with the vertical direction
    closest_axis_index = np.argmax(np.abs(cosines))
    return closest_axis_index

def get_ee_x_axis(transformation_matrix):
    # 提取末端执行器的X轴方向向量
    ee_x_axis = transformation_matrix[0:3, 0]
    return ee_x_axis

def align_ee_with_block(transformation_matrix, ee_matrix):
    rotation_matrix = transformation_matrix[:3, :3]
    vertical_direction = np.array([0, 0, -1])
    cosines = np.dot(rotation_matrix.T, vertical_direction)
    
    # 找到垂直轴
    vertical_axis_index = np.argmax(np.abs(cosines))
    
    # 选取一个非垂直轴，可以是X轴或Y轴，这里假设选择X轴
    target_axis_index = (vertical_axis_index + 1) % 3  # 简单循环选择一个非垂直轴
    target_axis = rotation_matrix[:, target_axis_index]
    
    # 末端执行器的X轴（假设开始时末端执行器X轴向前）
    ee_x_axis = ee_matrix[0:3, 0]
    
    # 计算旋转角度
    dot_product = np.dot(ee_x_axis, target_axis)
    angle = np.arccos(dot_product)  # 计算角度
    
    # 检查旋转方向（使用叉积的Z分量）
    cross_product = np.cross(ee_x_axis, target_axis)
    if cross_product[2] < 0:
        angle = -angle
    
    # 创建绕Z轴的旋转矩阵
    rot = R.from_rotvec(np.array([0, 0, angle]))
    rotation_matrix = rot.as_matrix()
    
    return rotation_matrix

def align_yaw_with_block(curr_yaw, blk_yaw):
    curr_yaw = (curr_yaw + np.pi) % (2 * np.pi) - np.pi
    blk_yaw = (blk_yaw + np.pi) % (2 * np.pi) - np.pi
    yaw_diff = blk_yaw - curr_yaw
    yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi
    min_rotation = np.round(yaw_diff / (np.pi / 2)) * (np.pi / 2)
    new_yaw = blk_yaw + min_rotation
    new_yaw = (new_yaw + np.pi) % (2 * np.pi) - np.pi
    return new_yaw

def scale_to_minus_pi_to_pi(angle):
    scaled_angle = angle % (2 * np.pi)
    if scaled_angle > np.pi:
        scaled_angle -= 2 * np.pi
    return scaled_angle

def get_ee_rotation_matrix(yaw_rate):
    rz = np.array([ [np.cos(yaw_rate),-np.sin(yaw_rate),0,0],
                    [np.sin(yaw_rate), np.cos(yaw_rate),0,0],
                    [0,                 0,              1,0],
                    [0,                 0,              0,1]])
    return rz

def get_target_joint_config(pos):
    target_joint_cfg, _, _, _ = ik.inverse(pos, seed, method='J_pseudo', alpha=0.5)
    for i in range(len(target_joint_cfg)):
        target_joint_cfg[i] = scale_to_minus_pi_to_pi(target_joint_cfg[i])
    target_joint_cfg[6] = target_joint_cfg[6] % pi

    # Avoid exceed joint limitations
    if target_joint_cfg[6] > 2.89730:
        target_joint_cfg[6] -= pi
    if target_joint_cfg[6] < -2.89730:
        target_joint_cfg[6] += pi
    
    return target_joint_cfg

if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")

    arm = ArmController()
    detector = ObjectDetector()

    # instantiate FK and IK
    fk = FK()
    ik = IK()
    seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    # start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    start_position = np.array([0, 0,  0, -pi/2, 0, pi/2, pi/4])
    _, start_T0e = fk.forward(start_position)
    start_T0e[0][3] -= 0.03
    start_T0e[1][3] -= 0.17
    start_T0e[2][3] += 0.2
    start_position, _, _, _ = ik.inverse(start_T0e, seed, method='J_trans', alpha=0.5)

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    #############
    ### START ###
    #############

    # STUDENT CODE HERE
    # static challenge
    arm.safe_move_to_position(start_position)
    _, start_T0e = fk.forward(start_position)

    # get the transform from camera to panda_end_effector
    H_ee_camera = detector.get_H_ee_camera()
    # pdb.set_trace()
    block_pos_list = []

    # Detect some blocks...
    for (name, pose) in detector.get_detections():
        # pdb.set_trace()
        print(name,'\n',start_T0e @ H_ee_camera @ pose)
        block_pos_list.append(start_T0e @ H_ee_camera @ pose)

    # #Uncomment to get middle camera depth/rgb images
    # mid_depth = detector.get_mid_depth()
    # mid_rgb = detector.get_mid_rgb()
    ee_yaw, ee_pitch, ee_roll = matrix_to_yaw_pitch_roll(start_T0e[:3, :3])
    prev_yaw = ee_yaw

    #Move around...
    arm.exec_gripper_cmd(0.12,50)
    pos_to_put_base = start_T0e
    pos_to_put_base[0,3] = 0.56
    pos_to_put_base[1,3] = 0.18
    pos_to_put_base[2,3] = 0.24
    print("The first block is going to be put at", pos_to_put_base)

    axis_list = ["x axis", "y axis", "z axis"]
    loop = 0

    for pos in block_pos_list:
        #################################################
        #### Get The Position of Block in Base Frame ####
        #################################################
        pitch_roll_yaw = matrix_to_yaw_pitch_roll(pos[:3, :3])
        print("block roll", pitch_roll_yaw[0])
        print("block pitch", pitch_roll_yaw[1])
        print("block yaw", pitch_roll_yaw[2])

        ######################################################################
        rotate_ee_matrix = align_ee_with_block(pos,start_T0e)
        new_ee_rotation_matrix = np.dot(rotate_ee_matrix, start_T0e[:3, :3])
        print("The adjusted new ee H matrix:", new_ee_rotation_matrix)
        pos[:3, :3] = new_ee_rotation_matrix
        ######################################################################

      # Down
        pos[2, 3] += 0.05
        target_joint_cfg = get_target_joint_config(pos)
        arm.safe_move_to_position(target_joint_cfg)
        
        # Grap
        pos[2, 3] -= 0.05
        target_joint_cfg = get_target_joint_config(pos)
        arm.safe_move_to_position(target_joint_cfg)
        arm.exec_gripper_cmd(0.025,50)
        
        # Up
        pos[2, 3] += 0.1
        target_joint_cfg = get_target_joint_config(pos)
        arm.safe_move_to_position(target_joint_cfg)

        # Move to target
        pos_to_put_base[2,3] += 0.1
        target_joint_cfg = get_target_joint_config(pos_to_put_base)
        arm.safe_move_to_position(target_joint_cfg)

        # Down
        pos_to_put_base[2,3] -= 0.1
        target_joint_cfg = get_target_joint_config(pos_to_put_base)
        arm.safe_move_to_position(target_joint_cfg)
        arm.exec_gripper_cmd(0.12,50)

        # Up and back to start
        pos_to_put_base[2,3] += 0.05
        target_joint_cfg = get_target_joint_config(pos_to_put_base)
        arm.safe_move_to_position(target_joint_cfg)
        print("move1 to", target_joint_cfg)
        arm.safe_move_to_position(start_position)
        print("move2 to", start_position)


    ############################
    ##### Dynamic Blocks #######
    ############################

    #
    # Create the position for ee to wait
    #

    H_rotate_90_y = np.array([[ 0,  0,  1,  0],
                              [ 0,  1,  0,  0],
                              [-1,  0,  0,  0],
                              [ 0,  0,  0,  1]])
    
    H_rotate_270_y = np.array([[ 0,  0, -1,  0],
                               [ 0,  1,  0,  0],
                               [ 1,  0,  0,  0],
                               [ 0,  0,  0,  1]])
    
    pos_to_wait_dynamic = H_rotate_90_y @ start_T0e    
    pos_to_wait_dynamic[0,3] = 0.05
    pos_to_wait_dynamic[1,3] = 0.65
    pos_to_wait_dynamic[2,3] = 0.215
    # 圆盘左侧一定距离，防止碰撞
    wait_position_safe, _, _, _ = ik.inverse(pos_to_wait_dynamic, seed, method='J_trans', alpha=0.5)
    # 向圆盘方向平移
    pos_to_wait_dynamic[1,3] = 0.69
    wait_position_for_block, _, _, _ = ik.inverse(pos_to_wait_dynamic, seed, method='J_trans', alpha=0.5)

    #######################
    ### Move and Place ####
    #######################
    while True:
        # safely move to prefixed position to wait for blks
        arm.safe_move_to_position(wait_position_safe)
        arm.safe_move_to_position(wait_position_for_block)

        # close the grip in each iteration
        # continue if no object gripped
        gripper_init_pos = 0.1
        for i in range(10):
            arm.exec_gripper_cmd(gripper_init_pos - i * 0.01)
            print(arm.get_gripper_state())


        # Move the object to the stack place 0.1m higher
        pos_to_put_base[2,3] += 0.1
        target_joint_cfg = get_target_joint_config(pos_to_put_base)
        arm.safe_move_to_position(target_joint_cfg)

        # Down
        pos_to_put_base[2,3] -= 0.1
        target_joint_cfg = get_target_joint_config(pos_to_put_base)
        arm.safe_move_to_position(target_joint_cfg)
        arm.exec_gripper_cmd(0.12,50)

        # Move to a safe place to go back to default
        pos_to_put_base[2,3] += 0.05
        target_joint_cfg = get_target_joint_config(pos_to_put_base)
        arm.safe_move_to_position(target_joint_cfg)
    #END STUDENT CODE
