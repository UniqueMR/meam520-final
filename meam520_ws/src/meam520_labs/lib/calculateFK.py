import math

import numpy as np
from math import pi


class FK():

    def __init__(self):
        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout

        pass

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here
        A1 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.141], [0, 0, 0, 1]])
        A2 = np.array(
            [[math.cos(q[0]), 0, -math.sin(q[0]), 0], [math.sin(q[0]), 0, math.cos(q[0]), 0], [0, -1, 0, 0.192],
             [0, 0, 0, 1]])
        A3 = np.array([[math.cos(q[1]), 0, math.sin(q[1]), 0.195 * math.sin(q[1])],
                       [math.sin(q[1]), 0, -math.cos(q[1]), -0.195 * math.cos(q[1])], [0, 1, 0, 0], [0, 0, 0, 1]])
        A4 = np.array([[math.cos(q[2]), 0, math.sin(q[2]), 0.0825 * math.cos(q[2])],
                       [math.sin(q[2]), 0, -math.cos(q[2]), 0.0825 * math.sin(q[2])], [0, 1, 0, 0.121], [0, 0, 0, 1]])
        A5 = np.array([[math.cos(q[3]), 0, -math.sin(q[3]), -0.0825 * math.cos(q[3]) - 0.125 * math.sin(q[3])],
                       [math.sin(q[3]), 0, math.cos(q[3]), -0.0825 * math.sin(q[3]) + 0.125 * math.cos(q[3])],
                       [0, -1, 0, 0], [0, 0, 0, 1]])
        A6 = np.array([[math.cos(q[4]), 0, math.sin(q[4]), -0.015 * math.sin(q[4])],
                       [math.sin(q[4]), 0, -math.cos(q[4]), 0.015 * math.cos(q[4])], [0, 1, 0, 0.259], [0, 0, 0, 1]])
        A7 = np.array([[math.cos(q[5]), 0, math.sin(q[5]), 0.088 * math.cos(q[5]) + 0.051 * math.sin(q[5])],
                       [math.sin(q[5]), 0, -math.cos(q[5]), 0.088 * math.sin(q[5]) - 0.051 * math.cos(q[5])],
                       [0, 1, 0, 0.015], [0, 0, 0, 1]])

        A8 = np.array([[math.cos(q[6] - pi / 4), -math.sin(q[6] - pi / 4), 0, 0],
                       [math.sin(q[6] - pi / 4), math.cos(q[6] - pi / 4), 0, 0], [0, 0, 1, 0.159], [0, 0, 0, 1]])
        jointPositions = np.zeros((8, 3))
        T0 = A1
        T01 = T0 @ A2
        T02 = T01 @ A3
        T03 = T02 @ A4
        T04 = T03 @ A5
        T05 = T04 @ A6
        T06 = T05 @ A7
        T0e = T06 @ A8

        # T0e = A1 @ A2 @ A3 @ A4 @ A5 @ A6 @ A7 @ A8
        jointPositions[0, :] = T0[:, 3][0:3]
        jointPositions[1, :] = T01[:, 3][0:3]
        jointPositions[2, :] = T02[:, 3][0:3]
        jointPositions[3, :] = T03[:, 3][0:3]
        jointPositions[4, :] = T04[:, 3][0:3]
        jointPositions[5, :] = T05[:, 3][0:3]
        jointPositions[6, :] = T06[:, 3][0:3]
        jointPositions[7, :] = T0e[:, 3][0:3]

        # Your code ends here

        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1

    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return ()

    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return ()


if __name__ == "__main__":
    fk = FK()

    # matches figure in the handout
    q = np.array([0, 0, 0, 0, 0, 0, 0])

    joint_positions, T0e = fk.forward(q)

    print("Joint Positions:\n", joint_positions)
    print("End Effector Pose:\n", T0e)
