
import sys
import numpy as np
from copy import deepcopy
from math import pi
import math
import pdb
import rospy
import sys
sys.path.append("../")
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector
from lib.calculateFK import FK
from lib.IK_position_null import IK
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

def matrix_to_yaw_pitch_roll(matrix):
    sy = np.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])

    singular = sy < 1e-6

    if not singular:
        yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
        pitch = np.arctan2(-matrix[2, 0], sy)
        roll = np.arctan2(matrix[2, 1], matrix[2, 2])
    else:
        yaw = np.arctan2(-matrix[1, 2], matrix[1, 1])
        pitch = np.arctan2(-matrix[2, 0], sy)
        roll = 0
    angle_set = [pitch,yaw,roll] # x,y,z
    return angle_set

def yaw_pitch_roll_to_matrix(yaw, pitch, roll):
    """Convert yaw, pitch, and roll angles to a 3x3 rotation matrix."""
    # Construct rotation matrix using yaw, pitch, and roll angles
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # Combine the rotation matrices
    rotation_matrix = np.dot(R_z, np.dot(R_y, R_x))

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

def get_ang(rot_idx, homo_mat):
    # find ms item
    ms_item, ms_val = 0, 0
    for i in range(3):
        if(np.abs(homo_mat[i][rot_idx]) > ms_val):
            ms_val = np.abs(homo_mat[i][rot_idx])
            ms_item = i

    valid_element = []

    for i in range(3):
        for j in range(3):
            if j != rot_idx and i!= ms_item:
                valid_element.append(homo_mat[i][j])

    return math.acos(np.abs(valid_element[0]))

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
        print("block pitch", pitch_roll_yaw[0])
        print("block roll", pitch_roll_yaw[1])
        print("block yaw", pitch_roll_yaw[2])

        # get the axis index that align with the ee z-axis
        vertical_axis_index = find_vertical_axis_and_adjust_yaw(pos)

        ang = get_ang(vertical_axis_index, pos)
        # pdb.set_trace()

        new_ee_yaw = align_yaw_with_block(prev_yaw, ang)
        print("new ee yaw: " + str(new_ee_yaw))
        prev_yaw = new_ee_yaw
        pos[:3, :3] = yaw_pitch_roll_to_matrix(new_ee_yaw, ee_pitch, ee_roll)

        # Down
        pos[2, 3] += 0.05
        target_joint_cfg = get_target_joint_config(pos)
        arm.safe_move_to_position(target_joint_cfg)
        
        # Grap
        pos[2, 3] -= 0.07
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
