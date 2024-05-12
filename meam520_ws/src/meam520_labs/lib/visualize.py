import numpy as np
from lib.calculateFK import FK


def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    J = np.zeros((6, 7))

    ## STUDENT CODE GOES HERE
    fk = FK()
    
    Jw = np.zeros((3, 7))
    Jv = np.zeros((3, 7))
    
    joint_positions, T0e = fk.forward(q_in)
    
    R0z = fk.get_axis_of_rotation(q_in)
    oe = T0e[0:3, -1]
    Jw = R0z
    
    i = 0
    while i < 7:
        Jv[:, i] = np.cross(R0z[:,i], (oe - joint_positions[i]))
        i += 1
    
    J = np.vstack((Jv, Jw))

    return J

if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q),3))
