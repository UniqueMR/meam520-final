import numpy as np
from lib.IK_velocity import IK_velocity
from lib.calcJacobian import calcJacobian

"""
Lab 3
"""

def IK_velocity_null(q_in, v_in, omega_in, b):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :param b: 7 x 1 Secondary task joint velocity vector
    :return:
    dq + null - 1 x 7 vector corresponding to the joint velocities + secondary task null velocities
    """

    ## STUDENT CODE GOES HERE
    dq = np.zeros((1, 7))
    null = np.zeros((1, 7))
    b = b.reshape((7, 1))
    v_in = np.array(v_in)
    v_in = v_in.reshape((3,1))
    omega_in = np.array(omega_in)
    omega_in = omega_in.reshape((3,1))

    v_all = np.vstack((v_in, omega_in))
    J = calcJacobian(q_in)
    
    # Check for NaN values in the velocity vector and adjust J and v_all accordingly
    for i in range(6): 
        if np.isnan(v_all[i][0]):
            J[i] = np.zeros((1, 7))
            v_all[i][0] = 0

    # pseudoinverse of J
    J_pinv = np.linalg.pinv(J)
    # null space projector of J
    N = np.eye(7) - np.dot(J_pinv, J)
    # primary task velocity
    dq = np.dot(J_pinv, v_all)
    # secondary task velocity in the null space
    null = np.dot(N, b)

    return dq.flatten() + null.flatten()
