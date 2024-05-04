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

        # targ_size = (2000, 1000)
        # calibrate_src_pts = np.array([[506, 227], [360, 176], [212, 220], [331, 285]], dtype=np.float32)
        # calibrate_targ_pts = np.array([[1305, 500], [1000, 195], [695, 500], [1000, 805]], dtype=np.float32)
        # self.trans_mat = cv2.getPerspectiveTransform(calibrate_src_pts, calibrate_targ_pts)
        # self.side_detector = sideCamDetector(self.trans_mat, targ_size)
        # self.side_camera_frame = np.array(None)
        # self.side_detection_stamp = None

        self.fk = FK()

        self.catch_trajectory = np.array([
            [ 0.6372539,   1.40164762,  0.80528983, -0.64127967,  0.60220947,  1.1247059,  -1.09397295 ],
            [ 0.89735778,  1.35316572,  0.81200166, -0.7687196,   0.69385432,  1.46417715, -1.10450269 ],
            [ 0.8755551,   1.21551891,  0.8406222,  -0.80134909,  0.7515986,   1.45789912, -0.99548829 ],
        ]) if team == 'red'\
        else np.array([
            [ 0.33499526, -1.44718133, -1.78532658, -1.29092163, -0.16000819,  1.65311508, -0.93638914 ],
            [ 0.84673118, -1.59435212, -1.63441296, -0.49283374, -0.32371925,  1.38519024, -0.91457699 ],
            [ 0.84214002, -1.48761881, -1.5852708,  -0.49118942, -0.29202438,  1.34041696, -0.80352218 ]
        ])

        self.put_trajectory = np.array([
            [
                [ 0.20647768,  0.10341451,  0.10881067, -1.8687268,  -0.0121758,   1.97153704, 1.10485937 ],
                [ 0.25655649,  0.19614435,  0.05678901, -2.01143984, -0.01375532,  2.20726171, 1.10583238 ],
                [ 0.22644902,  0.14137623,  0.08820374, -1.95011878, -0.01430496,  2.09093887, 1.10628493 ]
            ],
            [
                [ 0.19632443,  0.08340417,  0.11942184, -1.76600213, -0.01032211,  1.84882991, 1.10356607 ],
                [ 0.22644902,  0.14137623,  0.08820374, -1.95011878, -0.01430496,  2.09093887, 1.10628493 ],
                [ 0.20647768,  0.10341451,  0.10881067, -1.8687268,  -0.0121758,   1.97153704, 1.10485937 ]
            ],
            [
                [ 0.19625593,  0.08296527,  0.12055401, -1.63915905, -0.01008102,  1.72154515, 1.10331111 ],
                [ 0.20647768,  0.10341451,  0.10881067, -1.8687268,  -0.0121758,   1.97153704, 1.10485937 ],
                [ 0.19632443,  0.08340417,  0.11942184, -1.76600213, -0.01032211,  1.84882991, 1.10356607 ]
            ],
            [
                [ 0.20738953,  0.10459598,  0.11174524, -1.48310491, -0.01164338,  1.58711501, 1.10411573 ],
                [ 0.19632443,  0.08340417,  0.11942184, -1.76600213, -0.01032211,  1.84882991, 1.10356607 ],
                [ 0.19625593,  0.08296527,  0.12055401, -1.63915905, -0.01008102,  1.72154515, 1.10331111 ]
            ]
        ]) if team == 'red'\
        else np.array([
            [
                [-0.16139429,  0.15739791, -0.17193531, -1.28283998,  0.02708591,  1.43806041, 0.4577402  ],
                [-0.15130102,  0.08352683, -0.16783406, -1.63912252,  0.01412509,  1.72148428, 0.46471598 ],
                [-0.15386668,  0.10538884, -0.16954676, -1.48309612,  0.01777903,  1.58703699, 0.46261994 ]
            ],
            [
                [-0.18570343,  0.25390382, -0.16670467, -1.01011098,  0.04379911,  1.26128765, 0.4516074  ],
                [-0.15386668,  0.10538884, -0.16954676, -1.48309612,  0.01777903,  1.58703699, 0.46261994 ],
                [-0.16139429,  0.15739791, -0.17193531, -1.28283998,  0.02708591,  1.43806041, 0.4577402  ]
            ],
            [
                [-0.23533546,  0.38258236, -0.1347662,  -0.68306081,  0.05745935,  1.06640299, 0.45288853 ],
                [-0.16139429,  0.15739791, -0.17193531, -1.28283998,  0.02708591,  1.43806041, 0.4577402  ],
                [-0.18570343,  0.25390382, -0.16670467, -1.01011098,  0.04379911,  1.26128765, 0.4516074  ]
            ],
            [
                [-0.2703309,   0.43063683, -0.09004394, -0.50981428,  0.04666033,  0.94903617, 0.46075708 ],
                [-0.18570343,  0.25390382, -0.16670467, -1.01011098,  0.04379911,  1.26128765, 0.4516074  ],
                [-0.23533546,  0.38258236, -0.1347662,  -0.68306081,  0.05745935,  1.06640299, 0.45288853 ]
            ]
        ])


    def forward(self, put_idx):
        self.arm.open_gripper()
        for i in range(len(self.catch_trajectory)):
            self.arm.safe_move_to_position(self.catch_trajectory[i])
            if i == 1:        
                self.catch_block()

        for i in range(len(self.put_trajectory[0])):
            self.arm.safe_move_to_position(self.put_trajectory[put_idx][i])
            if i == 1:
                self.arm.open_gripper()

        
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

    for put_idx in range(4):
        dynamic_handler.forward(put_idx)

    

    


        



    
    