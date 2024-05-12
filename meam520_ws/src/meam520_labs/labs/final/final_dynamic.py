import sys
import numpy as np
from copy import deepcopy
from math import pi
import math
import pdb
import rospy
import sys
import scipy
from time import sleep
from dynamic import dynamicHandler

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

############# RED #######################


red_put_joint_cfg = np.array([[ 0.20468607,  0.09490256,  0.11311598, -1.88311308, -0.02228303,  1.99166204,  1.11157737],
                       [ 0.2569653,  0.18931933,  0.05859917, -2.02458586, -0.02604683,  2.22779257, 1.11560012],
                       [ 0.22552421,  0.13370667,  0.09153595, -1.96382906, -0.02543798,  2.11119227,  1.11449054],
                       [ 0.19394597,  0.07402191,  0.12436766, -1.78126469, -0.01969516,  1.86899738,  1.10896057],
                       [ 0.22552421,  0.13370667,  0.09153595, -1.96382906, -0.02543798,  2.11119227,  1.11449054],
                       [ 0.20468607,  0.09490256,  0.11311598, -1.88311308, -0.02228303,  1.99166204,  1.11157737],
                       [ 0.19345433,  0.07259674, 0.12598215, -1.65568089, -0.01906062,  1.74199775,  1.10755357],
                       [ 0.20468607,  0.09490256 , 0.11311598, -1.88311308, -0.02228303,  1.99166204,  1.11157737],
                       [ 0.19394597,  0.07402191,  0.12436766, -1.78126469, -0.01969516,  1.86899738,  1.10896057],
                       [ 0.20452827,  0.09338986,  0.11742127, -1.50082343, -0.0206406,   1.60786788, 1.10737957],
                       [ 0.19394597,  0.07402191,  0.12436766, -1.78126469, -0.01969516,  1.86899738,  1.10896057],
                       [ 0.19345433,  0.07259674, 0.12598215, -1.65568089, -0.01906062,  1.74199775,  1.10755357]])
red_wait_dynamic_joint_cfg = np.array([[ 0.63724355,  1.40161769,  0.80526806, -0.64131916,  0.60221379,  1.12474077, -1.09398961],
                                   [ 0.89734229,  1.35313286,  0.81199125, -0.76877188,  0.69385525,  1.46421889, -1.10451542]
                    ])

# ############## BLUE #######################
blue_put_joint_cfg = np.array([
                     ])

blue_wait_dynamic_joint_cfg = np.array([
                     ])
put_i = 0

###########################################
############### MAIN ######################
###########################################
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
    dyn = dynamicHandler(team)
    seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])


    if team == "red":
        put_joint_cfg = red_put_joint_cfg
        wait_dynamic_joint_cfg = red_wait_dynamic_joint_cfg
    else:
        put_joint_cfg = blue_put_joint_cfg
        wait_dynamic_joint_cfg = blue_wait_dynamic_joint_cfg


    start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    arm.safe_move_to_position(start_position) # on your mark!
    print('init 1: ', start_position)

    
    # Define where to set camera
    start_position_cfg = np.array([0, 0,  0, -pi/2, 0, pi/2, pi/4])
    ###################################################################################################################################
    _, start_T0e = fk.forward(start_position_cfg)
    # Default detection position
    # start_T0e[0][3] += 0.05 # simulationo
    start_T0e[0][3] -= 0.05 # real robot

    if team == "red":
        start_T0e[1][3] -= 0.17 ###RED
    else:
        start_T0e[1][3] += 0.17
        
    start_T0e[2][3] += 0.05
    ###################################################################################################################################
    default_start_position = start_T0e
    start_position_cfg, _, _, _ = ik.inverse(start_T0e, seed, method='J_trans', alpha=0.5)
    # start_position_cfg = np.array([0.22186471,  0.14780401,  0.08676945, -1.24896551, -0.01295858,  1.39628027, 1.09083977])
    print("start_position_cfg=",start_position_cfg)

    # Alter position 
    alter_start_cfg_list = []
    
    start_T0e[0][3] += 0.1
    alter_start_position_1 = start_T0e
    alter_start_cfg_1, _, _, _ = ik.inverse(alter_start_position_1, seed, method='J_trans', alpha=0.5)
    print("alter position 1: ", alter_start_cfg_1)
    # alter_start_cfg_1 = np.array([ 0.26837102,  0.58926434, -0.03274684, -0.60328531,  0.01959851,  1.19714571, 1.03378213])
    alter_start_cfg_list.append(alter_start_cfg_1)

    start_T0e[0][3] -= 0.2
    alter_start_position_2 = start_T0e
    alter_start_cfg_2, _, _, _ = ik.inverse(alter_start_position_2, seed, method='J_trans', alpha=0.5)
    print("alter position 2: ", alter_start_cfg_2)
    # alter_start_cfg_2 = np.array([ 0.07920207, -0.18704846,  0.24085819, -1.64480323,  0.04462776,  1.46310928, 1.1062142 ])
    alter_start_cfg_list.append(alter_start_cfg_2)

    start_T0e = default_start_position


    # Set speed
    arm.set_arm_speed(0.3)
    arm.set_gripper_speed(0.2)

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
    success_grip = 0
    detect_pos_i = 0
    

    # while True:
    #     # STUDENT CODE HERE
    #     # static challenge
    #     arm.safe_move_to_position(start_position_cfg)
    #     print('static 1: ', start_position_cfg)
    #     _, start_T0e = fk.forward(start_position_cfg)
    #     print("camera detection position is: \n", start_T0e)

    #     # get the transform from camera to panda_end_effector
    #     H_ee_camera = detector.get_H_ee_camera()
    #     print("H_ee_camera:\n", H_ee_camera)
    #     # pdb.set_trace()
    #     block_pos_list = []

    #     # Detect some blocks...
    #     print("====================================================")
    #     for (name, pose) in detector.get_detections():
    #         if team == "red":
    #             pose[0,3] += 0.01 # offset for x
    #             # pose[1,3] = 0.01 # offset for y 
    #         else:
    #             pose[0,3] += 0.00 # offset for x
    #             # pose[1,3] = 0.01 # offset for y 
    #         print(name,'\n',start_T0e @ H_ee_camera @ pose)
    #         block_pos_list.append(start_T0e @ H_ee_camera @ pose)

    #     # #Uncomment to get middle camera depth/rgb images
    #     # mid_depth = detector.get_mid_depth()
    #     # mid_rgb = detector.get_mid_rgb()

    #     #Move around...
    #     arm.open_gripper()

    # Define where to put the blocks
    pos_to_put_base = start_T0e
    pos_to_put_base[0,3] = 0.56
    if team == "red":
        pos_to_put_base[1,3] = 0.18
    else:
        pos_to_put_base[1,3] = -0.18
    pos_to_put_base[2,3] = 0.255
    print("The first block is going to be put at\n", pos_to_put_base)

    #     ############################
    #     ###### Static Blocks #######
    #     ############################

    #     for pos in block_pos_list:
    #         #################################################
    #         #### Get The Position of Block in Base Frame ####
    #         #################################################

    #         print("--------------------------------")
    #         print("----------New block-------------")
    #         print("--------------------------------")
    #         ######################################################################
    #         rotate_ee_matrix = align_ee_with_block(pos,start_T0e)
    #         new_ee_rotation_matrix = np.dot(rotate_ee_matrix, start_T0e[:3, :3])
    #         pos[:3, :3] = new_ee_rotation_matrix
    #         pos[2,3] = 0.275
    #         print("The adjusted position above this block is: \n", pos)
    #         ######################################################################

    #         # Down
    #         pos[2, 3] += 0.05
    #         target_joint_cfg = get_target_joint_config(pos)
    #         arm.safe_move_to_position(target_joint_cfg)
    #         print('static 2: ', target_joint_cfg)
            
    #         # Grap
    #         pos[2, 3] -= 0.08
    #         target_joint_cfg = get_target_joint_config(pos)
    #         arm.safe_move_to_position(target_joint_cfg)
    #         print('static 3: ', target_joint_cfg)
    #         arm.exec_gripper_cmd(0.025,75)

    #         # Get the gripper distance ang check state
    #         gripper_state = arm.get_gripper_state()
    #         gripper_positions = gripper_state['position']
    #         gripper_distance = gripper_positions[0] + gripper_positions[1]
    #         # If fail to grip
    #         if gripper_distance < 0.03:
    #             print("==============\n","Fail to grip\n","==============\n")
    #             start_position_cfg = alter_start_cfg_list[detect_pos_i]
    #             detect_pos_i +=1
    #             break
    #             # arm.safe_move_to_position(alter_start_cfg_1)
    #             # arm.open_gripper()
    #             # H_ee_camera = detector.get_H_ee_camera()
    #             # block_pos_list = []
    #             # for (name, pose) in detector.get_detections():
    #             #     pose[0,3] -= 0.05 # offset for x
    #             #     pose[1,3] -= 0.01 # offset for y 
    #             #     print(name,'\n',alter_start_position @ H_ee_camera @ pose)
    #             #     block_pos_list.append(alter_start_position @ H_ee_camera @ pose)
    #             # continue
            
    #         # Up
    #         pos[2, 3] += 0.1
    #         target_joint_cfg = get_target_joint_config(pos)
    #         arm.safe_move_to_position(target_joint_cfg)
    #         print('static 4: ', target_joint_cfg)

    #         # Move to target
    #         print("Move to target.....")
    #         pos_to_put_base[2,3] += 0.1
    #         target_joint_cfg = get_target_joint_config(pos_to_put_base)
    #         arm.safe_move_to_position(target_joint_cfg)
    #         print('static 5: ', target_joint_cfg)
    #         # arm.safe_move_to_position(put_joint_cfg[put_i])
    #         # put_i += 1


    #         # Down
    #         print("down.....")
    #         pos_to_put_base[2,3] -= 0.1
    #         target_joint_cfg = get_target_joint_config(pos_to_put_base)
    #         arm.safe_move_to_position(target_joint_cfg)
    #         print('static 6: ', target_joint_cfg)  
    #         # arm.safe_move_to_position(put_joint_cfg[put_i])
    #         # put_i += 1
    #         arm.open_gripper()
            

    #         # Up and back to start
    #         print("Up and back to start.....")
    #         pos_to_put_base[2,3] += 0.05
    #         target_joint_cfg = get_target_joint_config(pos_to_put_base)
    #         arm.safe_move_to_position(target_joint_cfg)
    #         print('static 7: ', target_joint_cfg)
    #         # arm.safe_move_to_position(put_joint_cfg[put_i])
    #         # put_i += 1

    #         arm.safe_move_to_position(start_position_cfg)
    #         print('static 8: ', start_position_cfg)
    #         success_grip +=1

    #     if success_grip == 4:
    #         print("--------------/n","Static Complete!", "------------------/n")
    #         break


                


    ############################
    ##### Dynamic Blocks #######
    ############################

    #
    # Create the position for ee to wait

    # for put_idx in range(4):
    #     dyn.forward(put_idx)
    # print("\n DYNAMIC 1 FInished \n")

    for i in range(4):

        arm.open_gripper()



        H_rotate_90_y = np.array([[ 0,  0,  1,  0],
                              [ 0,  1,  0,  0],
                              [-1,  0,  0,  0],
                              [ 0,  0,  0,  1]])
    
        H_rotate_270_y = np.array([[ 0,  0, -1,  0],
                               [ 0,  1,  0,  0],
                               [ 1,  0,  0,  0],
                               [ 0,  0,  0,  1]])

        H_rotate_180_z = np.array([[1,  0,  0,  0],
                               [ 0, -1,  0,  0],
                               [ 0,  0,  -1,  0],
                               [ 0,  0,  0,  1]])

        H_rotate_20_y = np.array([[0.940,  0,  0.342,  0],
                               [ 0, 1,  0,  0],
                               [ -0.342,  0,  0.940,  0],
                               [ 0,  0,  0,  1]])
        H_rotate_15_y = np.array([[0.966,  0,  0.259,  0],
                               [ 0, 1,  0,  0],
                               [ -0.259,  0,  0.966,  0],
                               [ 0,  0,  0,  1]])

        if team == "red":
            pos_to_wait_dynamic =  H_rotate_180_z @ H_rotate_90_y @ start_T0e
        else:  
            pos_to_wait_dynamic =  H_rotate_20_y @ H_rotate_270_y @ start_T0e

        if team == "red":
            pos_to_wait_dynamic[0,3] = 0.25
            pos_to_wait_dynamic[1,3] = 0.65         ###red
            pos_to_wait_dynamic[2,3] = 0.25
        else:
            pos_to_wait_dynamic[0,3] = -0.15
            pos_to_wait_dynamic[1,3] = -0.65        ###blue
            pos_to_wait_dynamic[2,3] = 0.21
        
        # 圆盘左侧一定距离，防止碰撞
        wait_position_safe, _, _, _ = ik.inverse(pos_to_wait_dynamic, seed, method='J_trans', alpha=0.5)
        arm.safe_move_to_position(wait_position_safe)
        print('dynamic 1: ', wait_position_safe)
        print("first step complete\n")
        # 向圆盘方向平移
        
        if team == "red":
            pos_to_wait_dynamic[0, 3] -= 0.2
            pos_to_wait_dynamic[1,3] += 0.1        ##red
        else:
            # pos_to_wait_dynamic[0, 3] += 0.05
            pos_to_wait_dynamic[1,3] -= 0.14       ##blue

        wait_position_for_block, _, _, _ = ik.inverse(pos_to_wait_dynamic, seed, method='J_trans', alpha=0.5)
        arm.safe_move_to_position(wait_position_for_block)
        print('dynamic 2: ', wait_position_for_block)
        

    #######################
    ### Move and Place ####
    #######################
    
        # safely move to prefixed position to wait for blks
       
        
        

        # close the grip in each iteration
        # continue if no object gripped
        # gripper_init_pos = arm.open_gripper()
        # print(gripper_init_pos)
        # for i in range(4):
        #     arm.exec_gripper_cmd(gripper_init_pos - i * 0.01)
        #     print(arm.get_gripper_state())

        # for i in range(5):
        while True:
            sleep(6)
            state = arm.exec_gripper_cmd(0.025, 100)
            gripper_state = arm.get_gripper_state()
            gripper_positions = gripper_state['position']
            gripper_distance = gripper_positions[0] + gripper_positions[1]
            if (gripper_distance > 0.055) or (gripper_distance < 0.03):
                print("nothing grabbed for " + str(i) + " times\n")
                print(f"Number of i: {i}")
                arm.open_gripper()
                continue
                
            else:
                print("successfully grabbed one block")
                print(f"Number of i: {i}")
                break
                

        # move up
        pos_to_wait_dynamic[2,3] += 0.1
        wait_position_for_block, _, _, _ = ik.inverse(pos_to_wait_dynamic, seed, method='J_trans', alpha=0.5)
        arm.safe_move_to_position(wait_position_for_block)
        print('dynamic 3: ', wait_position_for_block)

        # Move the object to the stack place 0.1m higher
        pos_to_put_base[2,3] += 0.1
        # target_joint_cfg = get_target_joint_config(pos_to_put_base)
        target_joint_cfg, _, _, _ = ik.inverse(pos_to_put_base, seed, method='J_trans', alpha=0.5)
        arm.safe_move_to_position(target_joint_cfg)
        print('dynamic 4: ', target_joint_cfg)

        # Down
        pos_to_put_base[2,3] -= 0.1
        # target_joint_cfg = get_target_joint_config(pos_to_put_base)
        target_joint_cfg, _, _, _ = ik.inverse(pos_to_put_base, seed, method='J_trans', alpha=0.5)
        arm.safe_move_to_position(target_joint_cfg)
        print('dynamic 5: ', target_joint_cfg)
        arm.open_gripper()

        # Move to a safe place to go back to default
        pos_to_put_base[2,3] += 0.05
        # target_joint_cfg = get_target_joint_config(pos_to_put_base)
        target_joint_cfg, _, _, _ = ik.inverse(pos_to_put_base, seed, method='J_trans', alpha=0.5)
        print('dynamic 6: ', target_joint_cfg)
        arm.safe_move_to_position(target_joint_cfg)
    print("End Loop")
    
    #END STUDENT CODE































    
