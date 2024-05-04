import numpy as np 
from lib.calcJacobian import calcJacobian



def IK_velocity(q_in, v_in, omega_in):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
         are infeasible, then dq should minimize the least squares error. If v_in
         and omega_in have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    ## STUDENT CODE GOES HERE

    dq = np.zeros((1, 7))

    v_in_column = v_in.reshape((3,1))
    omega_in_column = omega_in.reshape((3,1))
    
    Jacobian_matrix = calcJacobian(q_in)
    velocity_vector = np.vstack((v_in_column, omega_in_column))
    valid_rows = ~np.isnan(velocity_vector).ravel()
    nan_rows = np.isnan(velocity_vector).ravel()
    m = Jacobian_matrix[valid_rows, :]
    n = velocity_vector[valid_rows]
    dq, residuals, rank, s = np.linalg.lstsq(m, n, rcond=None)
    
    return dq
