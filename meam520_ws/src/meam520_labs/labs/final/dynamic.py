import sys
sys.path.append('../..')
from labs.side_camera.side_camera import sideCamDetector
from core.interfaces import ArmController, ObjectDetector
from lib.calculateFK import FK
from lib.IK_position_null import IK
import numpy as np
from math import pi
import rospy
from time import sleep
from matplotlib import pyplot as plt
import cv2
import pdb

class dynamicHandler:
    def __init__(self, team) -> None:
        self.team = team

        self.arm = ArmController()
        self.detector = ObjectDetector()

        targ_size = (2000, 1000)
        calibrate_src_pts = np.array([[506, 227], [360, 176], [212, 220], [331, 285]], dtype=np.float32)
        calibrate_targ_pts = np.array([[1305, 500], [1000, 195], [695, 500], [1000, 805]], dtype=np.float32)
        self.trans_mat = cv2.getPerspectiveTransform(calibrate_src_pts, calibrate_targ_pts)
        self.side_detector = sideCamDetector(self.trans_mat, targ_size)
        self.side_camera_frame = np.array(None)
        self.side_detection_stamp = None

        self.fk = FK()
        # self.ik = IK()
        self.end_effector_start = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
        _, self.start_T0e = self.fk.forward(self.end_effector_start)

        self.wait_position_safe = np.array([0.61653351, 1.31957225, 0.81314373, -0.68938005, 0.64868947, 1.12779263, -1.04486613]) if team == 'red' \
            else np.array([-0.59141293, 1.36660376, -0.94121216, -0.67323692, -0.53640283, 1.14407067, 2.57900007])
        self.wait_position_for_block = np.array([0.91472912, 1.32152006, 0.83423489, -0.7072616, 0.7070031, 1.4366291, -1.02527523]) if team == 'red' \
            else np.array([-0.59141293, 1.36660376, -0.94121216, -0.67323692, -0.53640283, 1.14407067, 2.57900007])
        
        # self.put_position_base = self.start_T0e
        # self.put_position_base[0:3, 3] = (0.56, 0.18, 0.255)
        self.put_joint_cfg = np.array([

        ])


    def forward(self, put_idx):
        print('move to start position ... ')
        self.arm.safe_move_to_position(self.end_effector_start)
        self.arm.open_gripper()
        print('move to the position to catch dynamic blocks ... ')

        # detect the position of dynamic blocks
        frame = self.update_side_camera_frame()
        self.side_detection_stamp = self.side_detector.side_camera_detection(frame, self.team)
        target_r = self.find_block_with_min_dist()
        pdb.set_trace()
        self.arm.safe_move_to_position(self.wait_position_safe)
        self.arm.safe_move_to_position(self.wait_position_for_block)
        self.catch_block()
        self.arm.safe_move_to_position(self.wait_position_safe)
        self.arm.safe_move_to_position(self.end_effector_start)

        # pos_to_put = self.put_position_base
        # pos_to_put[2, 3] += z_to_put
        # joint_to_put, _, _, _ = self.ik.inverse(pos_to_put, self.end_effector_start, method='J_trans', alpha=0.5)
        
        self.arm.safe_move_to_position(self.put_joint_cfg[put_idx])

    def catch_block(self, lb=0.03, ub=0.055):
        while True:
            sleep(6)
            state = self.arm.exec_gripper_cmd(0.03, 120)
            gripper_state = self.arm.get_gripper_state()
            gripper_positions = gripper_state['position']
            gripper_distance = gripper_positions[0] + gripper_positions[1]
            if (gripper_distance > ub) or (gripper_distance < lb):
                self.arm.open_gripper()
            else:
                break
        return

    def update_side_camera_frame(self):
        frame = np.array(None)
        while True:
            frame = np.array(self.detector.get_mid_rgb())
            if frame.any() == None:
                continue
            self.side_camera_frame = frame
            break
        return self.side_camera_frame

    def find_block_with_min_dist(self):
        filtered_detections = []
        for detection in self.side_detection_stamp:
            if self.team == 'blue' and detection['position'] < 0:
                continue
            if self.team == 'red' and detection['position'] > 0:
                continue
            filtered_detections.append(detection)

        if filtered_detections.empty():
            return -1
        
        target = min(filtered_detections, key=lambda detection: detection['position'][0] ** 2 + detection['position'][1] ** 2)
        return target['r']
    
if __name__ == '__main__':
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")

    dynamic_handler = dynamicHandler(team)

    put_idx = 0
    while True:
        dynamic_handler.forward(put_idx)
        put_idx += 1

    

    


        



    
    