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

    Jacobian = calcJacobian(q_in)

    y = np.concatenate([v_in,omega_in])

    nanvec = []

    v = np.array([])
    J = np.array([])

    for i in range(6):
        if not np.isnan(y[i]):
            v = np.append(v,[y[i]])
            J = np.append(J, Jacobian[i])

    # for item in nanvec:
    #     y[item] = 0
    #     Jacobian[item] = np.zeros(7)

    # y = y.T
    v = v.T
    J = J.reshape(-1,7)


    dq = np.linalg.lstsq(J, v, rcond=None)[0]

    # v_in = v_in.reshape((3,1))
    # omega_in = omega_in.reshape((3,1))
    
    return dq


q_in = np.array([0,0,0,0,0,np.pi,0])
v_in = np.array([0.5,0.5,np.nan])
omega_in = np.array([0.5,0.5,0.5])
print(IK_velocity(q_in,v_in,omega_in))