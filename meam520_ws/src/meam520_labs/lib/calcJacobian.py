import numpy as np
from lib.calculateFK import FK

fk = FK()

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

    Jv = np.zeros((3, 7))
    Jw = np.zeros((3, 7))

    # Calculate joint positions and the end effector pose using forward kinematics.
    joint_positions, T0e = fk.forward(q_in)

    # Get the rotation axes for all joints.
    z_axis = fk.get_axis_of_rotation(q_in)

    # End effector positions
    o_vectors = T0e[0:3, -1]

    for i in range(7):
        # The angular velocity is directly given by the rotation axis for revolute joints.
        Jw[:, i] = z_axis[:, i]

        # Using the cross product to calculate linear velocity
        Jv[:, i] = np.cross(z_axis[:, i], (o_vectors - joint_positions[i, :]))

        J = np.vstack([Jv, Jw])
    
    return J

if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q),3))
