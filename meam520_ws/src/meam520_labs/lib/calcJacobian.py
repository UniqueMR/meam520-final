import numpy as np
# from lib.calculateFK import FK
from math import pi
from math import sin
from math import cos
from math import sqrt
import math


def dotfun(matrix_vector):
    res = matrix_vector[0]
    for i in range(1, len(matrix_vector)):
        res = np.dot(res, matrix_vector[i])
    return res


def calcJacobian(q):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    # jointPositions = np.zeros((8, 3))
    # T0e = np.identity(4)

    # Your code ends here
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
    p0 = T0[:, 3][0:3]
    p1 = T01[:, 3][0:3]
    p2 = T02[:, 3][0:3]
    p3 = T03[:, 3][0:3]
    p4 = T04[:, 3][0:3]
    p5 = T05[:, 3][0:3]
    p6 = T06[:, 3][0:3]
    p7 = T0e[:, 3][0:3]

    z0 = T0[:, 2][0:3]
    z1 = T01[:, 2][0:3]
    z2 = T02[:, 2][0:3]
    z3 = T03[:, 2][0:3]
    z4 = T04[:, 2][0:3]
    z5 = T05[:, 2][0:3]
    z6 = T06[:, 2][0:3]
    # z7 = T0e[:, 2][0:3]

    Jv0 = np.cross(z0, (p7 - p0) )
    Jv1 = np.cross(z1, (p7 - p1) )
    Jv2 = np.cross(z2, (p7 - p2) )
    Jv3 = np.cross(z3, (p7 - p3) )
    Jv4 = np.cross(z4, (p7 - p4) )
    Jv5 = np.cross(z5, (p7 - p5) )
    Jv6 = np.cross(z6, (p7 - p6) )

    J = np.zeros((6, 7))

    J[0:3][:, 0] = Jv0
    J[0:3][:, 1] = Jv1
    J[0:3][:, 2] = Jv2
    J[0:3][:, 3] = Jv3
    J[0:3][:, 4] = Jv4
    J[0:3][:, 5] = Jv5
    J[0:3][:, 6] = Jv6
    J[0:3][:, 0] = Jv0

    J[3:6][:, 0] = z0.T
    J[3:6][:, 1] = z1.T
    J[3:6][:, 2] = z2.T
    J[3:6][:, 3] = z3.T
    J[3:6][:, 4] = z4.T
    J[3:6][:, 5] = z5.T
    J[3:6][:, 6] = z6.T

    ## STUDENT CODE GOES HERE

    return J


if __name__ == '__main__':
    # q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    q1 = np.array([0, 0, 0, 0, 0, 0, 1])
    print(np.round(calcJacobian(q1), 3))

