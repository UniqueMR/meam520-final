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
# from dynamic import dynamicHandler

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


red_put_joint_cfg = np.array([[ 0.20637423,  0.10329483,  0.10891385, -1.86892883, -0.01218607,  1.97160363,  1.10486544],
                        [ 0.25646696,  0.19604132,  0.05688215, -2.01160764, -0.0137859,   2.20730894,  1.10585503],
                        [ 0.22635106,  0.14126689,  0.08830361, -1.95030018, -0.01432389,  2.09099407,  1.10629819],
                        [ 0.19621604,  0.08327022,  0.11952764, -1.76623226, -0.01032689,  1.84891138,  1.10356733],
                        [ 0.22635106,  0.14126689,  0.08830361, -1.95030018, -0.01432389,  2.09099407,  1.10629819],
                        [ 0.20637423,  0.10329483,  0.10891385, -1.86892883, -0.01218607,  1.97160363,  1.10486544],
                        [ 0.196135,    0.08280473,  0.12067022, -1.63943983, -0.01008311,  1.72165196,  1.10330974],
                        [ 0.20637423,  0.10329483,  0.10891385, -1.86892883, -0.01218607,  1.97160363,  1.10486544],
                        [ 0.19621604,  0.08327022,  0.11952764, -1.76623226, -0.01032689,  1.84891138,  1.10356733],
                        [ 0.20749655,  0.10485439,  0.11166051, -1.48265605, -0.01167617,  1.58687543,  1.10413518],
                        [ 0.19621604,  0.08327022,  0.11952764, -1.76623226, -0.01032689,  1.84891138,  1.10356733],
                        [ 0.196135,    0.08280473,  0.12067022, -1.63943983, -0.01008311,  1.72165196,  1.10330974],
                        
                        [ 0.23318306,  0.15594956,  0.08935327, -1.28283558, -0.01399462,  1.43822179,  1.10500565],
                        [ 0.19517652,  0.08092882,  0.12120827, -1.67997315, -0.00996594,  1.76029966,  1.1032685 ],
                        [ 0.20266186,  0.0953778,   0.11561924, -1.53378611, -0.01101618,  1.62855955,  1.1037943 ],
                        [ 0.26142653,  0.21722423,  0.06217743, -1.10147798, -0.0138417,   1.31846478,  1.10408661],
                        [ 0.19910398,  0.08847341,  0.11844995, -1.58117669, -0.01050365,  1.66905966,  1.10352349],
                        [ 0.21574856,  0.12098904,  0.10471153, -1.4094544,  -0.01263738,  1.52982409,  1.10458052],
                        [ 0.29352253,  0.3048346,   0.02571085, -0.87555094, -0.00835482,  1.18124035,  1.10026626],
                        [ 0.20926629,  0.10825909,  0.11018789, -1.46534407, -0.01189317,  1.57298449,  1.10423834],
                        [ 0.23673023,  0.1632538,   0.08610952, -1.25934019, -0.0141464,   1.42205088,  1.10500179],
                        [ 0.31289121,  0.39229061, -0.00347571, -0.65332146,  0.0015223,   1.04930288,  1.09584151],
                        [ 0.22666117,  0.14271278,  0.09520836, -1.32757676, -0.01360237,  1.4696962,  1.10493246],
                        [ 0.26604078,  0.22817604,  0.05739016, -1.07185291, -0.01347761,  1.29988198,  1.10373512]

])

red_wait_dynamic_joint_cfg = np.array([[ 0.6587729,   1.24464193,  0.6724978,  -0.78233758,  0.42564804,  1.29159651, -1.2318193 ],
                        [ 0.9221681,   1.2288717,   0.71871254, -0.87171266,  0.49760699,  1.6069969, -1.12734984],
                        [ 0.91192979,  1.11144085,  0.74691449, -0.85142519,  0.53602883,  1.54009243, -1.01464821]
                    ])

# ############## BLUE #######################
blue_put_joint_cfg = np.array([[-0.14587314,  0.1041857,  -0.17193363, -1.86890578,  0.0193371,   1.97151264, 0.46096142],
                        [-0.13289403,  0.19904026, -0.18586264, -2.01142893,  0.04544459,  2.20664875,  0.44322944],
                        [-0.14045058,  0.14291017, -0.17797128, -1.95023754,  0.0290708,   2.09075169,  0.45430617],
                        [-0.14925166,  0.08384137, -0.16846337, -1.76622065,  0.01461438,  1.84886597, 0.46425147],
                        [-0.14045058,  0.14291017, -0.17797128, -1.95023754,  0.0290708,   2.09075169,  0.45430617],
                        [-0.14587314,  0.1041857,  -0.17193363, -1.86890578,  0.0193371,   1.97151264,  0.46096142],
                        [-0.15133209,  0.08334736, -0.16777278, -1.63942903,  0.0140735,   1.72161029,  0.46474987],
                        [-0.14587314,  0.1041857,  -0.17193363, -1.86890578,  0.0193371,   1.97151264,  0.46096142],
                        [-0.14925166,  0.08384137, -0.16846337, -1.76622065,  0.01461438,  1.84886597,  0.46425147],
                        [-0.15391049,  0.10565603, -0.16953433, -1.4826362,   0.01780826,  1.58679949,  0.46259548],
                        [-0.14925166,  0.08384137, -0.16846337, -1.76622065,  0.01461438,  1.84886597,  0.46425147],
                        [-0.15133209,  0.08334736, -0.16777278, -1.63942903,  0.0140735,   1.72161029,  0.46474987],

                        [-0.16139156,  0.15741904, -0.17193565, -1.28280657,  0.02707408,  1.43802606,  0.45774175],
                        [-0.15072995,  0.08146512, -0.16775464, -1.67994808,  0.01384802,  1.76025332,  0.46485021],
                        [-0.15286134,  0.09604968, -0.16881655, -1.53379286,  0.01615111,  1.62851015,  0.46355254],
                        [-0.1755036,   0.21926909, -0.17017414, -1.10162776,  0.03806357,  1.31815609,  0.45324499],
                        [-0.15210763,  0.08906808, -0.16824042, -1.58117999,  0.0149777,   1.66901774,  0.46423611],
                        [-0.15585236,  0.12198691, -0.17064338, -1.4094504,   0.02069359,  1.52972533,  0.46099667],
                        [-0.20388366,  0.30655871, -0.15769194, -0.8767711,   0.05127268,  1.18117942,  0.45060058],
                        [-0.15427699,  0.10909188, -0.16981461, -1.46534404,  0.01841157,  1.57291164,  0.46225849],
                        [-0.16278321,  0.16482055, -0.17200638, -1.25930368,  0.02841236,  1.42183069,  0.45711361],
                        [-0.24007309,  0.39258398, -0.13037131, -0.65585129,  0.05751954,  1.04986265,  0.45360798],
                        [-0.15907624,  0.14400225, -0.17162702, -1.32756129,  0.02464809,  1.46954221,  0.45892983],
                        [-0.17857268,  0.23026494, -0.16927532, -1.07207798,  0.0399291,   1.29957123,  0.45264848]
                     ])

blue_wait_dynamic_joint_cfg = np.array([[ 0.33816057, -1.45816366, -1.79439962, -1.28800824, -0.16636337,  1.65744569, -0.94700854],
                        [ 0.88958606, -1.59881336, -1.66422115, -0.48825645, -0.29754798,  1.42829993, -0.91764978],
                        [ 0.8844535,  -1.49346762, -1.61317865, -0.48761137, -0.27356506,  1.38345539, -0.80783083]
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
    # dyn = dynamicHandler(team)
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
    # start_T0e[0][3] += 0.05 # simulation
    start_T0e[0][3] -= 0.05 # real robot

    if team == "red":
        start_T0e[1][3] -= 0.17 ###RED
    else:
        start_T0e[1][3] += 0.17
        
    start_T0e[2][3] += 0.05
    ###################################################################################################################################
    default_start_position = start_T0e
    # start_position_cfg, _, _, _ = ik.inverse(start_T0e, seed, method='J_trans', alpha=0.5)
    # print("start_position_cfg=",start_position_cfg)
    if team == "red":
        start_position_cfg = np.array([-0.16370853, -0.02778733, -0.15759993, -1.47168115, -0.00439685,  1.44425791,  0.46359517])
    else:
        start_position_cfg = np.array([ 0.14737862, -0.02785091,  0.17354891, -1.47165929,  0.00484856,  1.44424577, 1.10687114])

    # Alter position 
    alter_start_cfg_list = []
    
    start_T0e[0][3] += 0.1
    alter_start_position_1 = start_T0e
    # alter_start_cfg_1, _, _, _ = ik.inverse(alter_start_position_1, seed, method='J_trans', alpha=0.5)
    # print("alter position 1: ", alter_start_cfg_1)
    if team == "red":
        alter_start_cfg_1 = np.array([-0.17738912,  0.36508447, -0.14503989, -0.94158462,  0.05352144,  1.30412555,  0.486557  ])
    else:
        alter_start_cfg_1 = np.array([ 2.73062371e-01,  3.63008585e-01,  1.61851543e-03, -9.40962033e-01, -5.95731198e-04,  1.30455026e+00,  1.05981647e+00])
    alter_start_cfg_list.append(alter_start_cfg_1)

    start_T0e[0][3] -= 0.2
    alter_start_position_2 = start_T0e
    # alter_start_cfg_2, _, _, _ = ik.inverse(alter_start_position_2, seed, method='J_trans', alpha=0.5)
    # print("alter position 2: ", alter_start_cfg_2)
    if team == "red":
        alter_start_cfg_2 = np.array([-0.23646275, -0.32689281, -0.12587776, -1.78812591, -0.04055528,  1.46371393,  0.42532196])
    else:
        alter_start_cfg_2 = np.array([ 0.03218045, -0.3406194,   0.28310195, -1.78716015,  0.094034,    1.45960519,  1.0956675 ])
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
    

    while True:
        # STUDENT CODE HERE
        # static challenge
        arm.safe_move_to_position(start_position_cfg)
        print('static 1: ', start_position_cfg)
        _, start_T0e = fk.forward(start_position_cfg)
        print("camera detection position is: \n", start_T0e)

        # get the transform from camera to panda_end_effector
        H_ee_camera = detector.get_H_ee_camera()
        print("H_ee_camera:\n", H_ee_camera)
        # pdb.set_trace()
        block_pos_list = []

        # Detect some blocks...
        print("====================================================")
        for (name, pose) in detector.get_detections():
            if team == "red":
                pose[0,3] -= 0.01  # offset for x
                pose[1,3] -= 0.01 # offset for y  
            else:
                pose[0,3] += 0.00 # offset for x
                # pose[1,3] = 0.01 # offset for y 
            print(name,'\n',start_T0e @ H_ee_camera @ pose)
            block_pos_list.append(start_T0e @ H_ee_camera @ pose)

        # #Uncomment to get middle camera depth/rgb images
        # mid_depth = detector.get_mid_depth()
        # mid_rgb = detector.get_mid_rgb()

        #Move around...
        arm.open_gripper()

        # Define where to put the blocks
        pos_to_put_base = start_T0e
        pos_to_put_base[0,3] = 0.56
        if team == "red":
            pos_to_put_base[1,3] = 0.18
        else:
            pos_to_put_base[1,3] = -0.18
        pos_to_put_base[2,3] = 0.255
        print("The first block is going to be put at\n", pos_to_put_base)

        ############################
        ###### Static Blocks #######
        ############################

        for pos in block_pos_list:
            #################################################
            #### Get The Position of Block in Base Frame ####
            #################################################

            print("--------------------------------")
            print("----------New block-------------")
            print("--------------------------------")
            ######################################################################
            rotate_ee_matrix = align_ee_with_block(pos,start_T0e)
            new_ee_rotation_matrix = np.dot(rotate_ee_matrix, start_T0e[:3, :3])
            pos[:3, :3] = new_ee_rotation_matrix
            pos[2,3] = 0.275
            print("The adjusted position above this block is: \n", pos)
            ######################################################################

            # Down
            pos[2, 3] += 0.05
            target_joint_cfg = get_target_joint_config(pos)
            arm.safe_move_to_position(target_joint_cfg)
            print('static 2: ', target_joint_cfg)
            
            # Grap
            pos[2, 3] -= 0.08
            target_joint_cfg = get_target_joint_config(pos)
            arm.safe_move_to_position(target_joint_cfg)
            print('static 3: ', target_joint_cfg)
            arm.exec_gripper_cmd(0.025,75)

            # Get the gripper distance ang check state
            gripper_state = arm.get_gripper_state()
            gripper_positions = gripper_state['position']
            gripper_distance = gripper_positions[0] + gripper_positions[1]
            # If fail to grip
            if gripper_distance < 0.03:
                print("==============\n","Fail to grip\n","==============\n")
                start_position_cfg = alter_start_cfg_list[detect_pos_i]
                detect_pos_i +=1
                break
                # arm.safe_move_to_position(alter_start_cfg_1)
                # arm.open_gripper()
                # H_ee_camera = detector.get_H_ee_camera()
                # block_pos_list = []
                # for (name, pose) in detector.get_detections():
                #     pose[0,3] -= 0.05 # offset for x
                #     pose[1,3] -= 0.01 # offset for y 
                #     print(name,'\n',alter_start_position @ H_ee_camera @ pose)
                #     block_pos_list.append(alter_start_position @ H_ee_camera @ pose)
                # continue
            
            # Up
            pos[2, 3] += 0.1
            target_joint_cfg = get_target_joint_config(pos)
            arm.safe_move_to_position(target_joint_cfg)
            print('static 4: ', target_joint_cfg)

            # Move to target
            # print("Move to target.....")
            # pos_to_put_base[2,3] += 0.1
            # target_joint_cfg = get_target_joint_config(pos_to_put_base)
            # arm.safe_move_to_position(target_joint_cfg)
            # print('static 5: ', target_joint_cfg)
            arm.safe_move_to_position(put_joint_cfg[put_i])
            put_i += 1


            # Down
            # print("down.....")
            # pos_to_put_base[2,3] -= 0.1
            # target_joint_cfg = get_target_joint_config(pos_to_put_base)
            # arm.safe_move_to_position(target_joint_cfg)
            # print('static 6: ', target_joint_cfg)  
            arm.safe_move_to_position(put_joint_cfg[put_i])
            put_i += 1
            arm.open_gripper()
            

            # Up and back to start
            # print("Up and back to start.....")
            # pos_to_put_base[2,3] += 0.05
            # target_joint_cfg = get_target_joint_config(pos_to_put_base)
            # arm.safe_move_to_position(target_joint_cfg)
            # print('static 7: ', target_joint_cfg)
            arm.safe_move_to_position(put_joint_cfg[put_i])
            put_i += 1

            arm.safe_move_to_position(start_position_cfg)
            print('static 8: ', start_position_cfg)
            success_grip +=1

        if success_grip == 4:
            arm.safe_move_to_position(start_position)
            print("--------------/n","Static Complete!", "------------------/n")
            break


                


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
        H_rotate_10_y = np.array([[0.985,  0,  0.174,  0],
                               [ 0, 1,  0,  0],
                               [ -0.174,  0,  0.985,  0],
                               [ 0,  0,  0,  1]])
        H_rotate_minus_20_y = np.array([[0.940,  0,  -0.342,  0],
                               [ 0, 1,  0,  0],
                               [ 0.342,  0,  0.940,  0],
                               [ 0,  0,  0,  1]])

        if team == "red":
            pos_to_wait_dynamic =  H_rotate_minus_20_y @ H_rotate_180_z @ H_rotate_90_y @ start_T0e
        else:  
            pos_to_wait_dynamic =  H_rotate_20_y @ H_rotate_270_y @ start_T0e

        if team == "red":
            pos_to_wait_dynamic[0,3] = 0.25
            pos_to_wait_dynamic[1,3] = 0.65         ###red
            pos_to_wait_dynamic[2,3] = 0.214
        else:
            pos_to_wait_dynamic[0,3] = -0.15
            pos_to_wait_dynamic[1,3] = -0.65        ###blue
            pos_to_wait_dynamic[2,3] = 0.21
        
        # To the left of the turntable to prevent collisions
        # wait_position_safe, _, _, _ = ik.inverse(pos_to_wait_dynamic, seed, method='J_trans', alpha=0.5)
        # arm.safe_move_to_position(wait_position_safe)
        # print('dynamic 1: ', wait_position_safe)
        arm.safe_move_to_position(wait_dynamic_joint_cfg[0])
        print("first step complete\n")
        
        # toward the turntable
        if team == "red":
            pos_to_wait_dynamic[0, 3] -= 0.2
            pos_to_wait_dynamic[1,3] += 0.1        ##red
        else:
            pos_to_wait_dynamic[0, 3] += 0.025
            pos_to_wait_dynamic[1,3] -= 0.16       ##blue

        # wait_position_for_block, _, _, _ = ik.inverse(pos_to_wait_dynamic, seed, method='J_trans', alpha=0.5)
        # arm.safe_move_to_position(wait_position_for_block)
        # print('dynamic 2: ', wait_position_for_block)
        arm.safe_move_to_position(wait_dynamic_joint_cfg[1])
        
        

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
        # pos_to_wait_dynamic[2,3] += 0.1
        # wait_position_for_block, _, _, _ = ik.inverse(pos_to_wait_dynamic, seed, method='J_trans', alpha=0.5)
        # arm.safe_move_to_position(wait_position_for_block)
        # print('dynamic 3: ', wait_position_for_block)
        arm.safe_move_to_position(wait_dynamic_joint_cfg[2])

        # Move the object to the stack place 0.1m higher
        # pos_to_put_base[2,3] += 0.1
        # target_joint_cfg, _, _, _ = ik.inverse(pos_to_put_base, seed, method='J_trans', alpha=0.5)
        # arm.safe_move_to_position(target_joint_cfg)
        # print('dynamic 4: ', target_joint_cfg)
        arm.safe_move_to_position(put_joint_cfg[put_i])
        put_i += 1

        # Down
        # pos_to_put_base[2,3] -= 0.11
        # target_joint_cfg, _, _, _ = ik.inverse(pos_to_put_base, seed, method='J_trans', alpha=0.5)
        # arm.safe_move_to_position(target_joint_cfg)
        # print('dynamic 5: ', target_joint_cfg)
        arm.safe_move_to_position(put_joint_cfg[put_i])
        put_i += 1
        arm.open_gripper()

        # Move to a safe place to go back to default
        # pos_to_put_base[2,3] += 0.05
        # target_joint_cfg, _, _, _ = ik.inverse(pos_to_put_base, seed, method='J_trans', alpha=0.5)
        # print('dynamic 6: ', target_joint_cfg)
        # arm.safe_move_to_position(target_joint_cfg)
        arm.safe_move_to_position(put_joint_cfg[put_i])
        put_i += 1

    print("End Loop")
    
    #END STUDENT CODE