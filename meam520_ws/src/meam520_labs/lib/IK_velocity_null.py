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


    #primary_joint_velocities = IK_velocity(q_in, v_in, omega_in)

    J = calcJacobian(q_in)

    # Create target velocity vector and filter out NaN values
    target_velocity = np.vstack((v_in, omega_in))
    #valid_indices = ~np.isnan(target_velocity).flatten()
    #J_filtered = J[valid_indices, :]

    for i in range(6): 
        if np.isnan(target_velocity[i][0]):
            J[i] = np.zeros((1, 7))
            target_velocity[i][0] = 0


    J_pseudo_inverse = np.linalg.pinv(J)
    I = np.eye(J.shape[1])

    # Compute secondary task velocities projected onto the null space of the Jacobian
    null_space_projector = I - J_pseudo_inverse@ J
    dq = np.dot(J_pseudo_inverse, target_velocity)
    secondary_joint_velocity = np.dot(null_space_projector, b)


    joint_velocities = dq.flatten() + secondary_joint_velocity.flatten()

    return joint_velocities








    return dq + null

