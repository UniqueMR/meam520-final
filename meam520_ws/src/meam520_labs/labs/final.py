import sys
import numpy as np
from copy import deepcopy
from math import pi
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

    return yaw, pitch, roll

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
    arm.safe_move_to_position(start_position) # on your mark!
    _, start_T0e = fk.forward(start_position)

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE

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

    loop = 0
    for pos in block_pos_list:
        #################################################
        #### Get The Position of Block in Base Frame ####
        #################################################
        blk_yaw, blk_pitch, blk_roll = matrix_to_yaw_pitch_roll(pos[:3, :3])
        print("block yaw" + str(blk_yaw))
        print("pitch" + str(blk_pitch))
        print("roll" + str(blk_roll))
        
        new_ee_yaw = align_yaw_with_block(prev_yaw, blk_yaw)
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
        pos_to_put_base[2,3] += 0.06
        target_joint_cfg = get_target_joint_config(pos_to_put_base)
        arm.safe_move_to_position(target_joint_cfg)
        print("move1 to", target_joint_cfg)
        arm.safe_move_to_position(start_position)
        print("move2 to", start_position)
        
    #END STUDENT CODE