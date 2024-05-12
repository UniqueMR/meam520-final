import sys
import numpy as np
from copy import deepcopy
from math import pi
import pdb
import rospy
import sys
sys.path.append("../..")
from core.interfaces import ArmController
from core.interfaces import ObjectDetector
from core.utils import time_in_seconds
import imageio


if __name__ == "__main__":

    try:
        assert sys.argv[1] == 'calibrate' or sys.argv[1] == 'sample', 'unrecognized mode'
    except AssertionError as error:
        print(f"Error: {error}")
        exit()

    try:
        if sys.argv[1] == 'calibrate':
            assert len(sys.argv) == 2, 'unrecognized param'
        else:
            assert len(sys.argv) == 3, 'cannot find num of samples'
    except AssertionError as error:
        print(f'Error: {error}')
        exit()

    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    

    arm = ArmController()
    detector = ObjectDetector()

    num_of_samples = 1 if sys.argv[1] == 'calibrate' else int(sys.argv[2])
    cnt = 0
    while True:
        mid_rgb = np.array(detector.mid_rgb)
        if mid_rgb.any() == None:
            continue
        if sys.argv[1] == 'calibrate':
            imageio.imwrite('./imgs/calibrate/calibrate.png', mid_rgb)
        else:
            imageio.imwrite('./imgs/samples/sample_' + str(cnt) +'.png', mid_rgb)
        cnt += 1
        if cnt >= num_of_samples:
            break
