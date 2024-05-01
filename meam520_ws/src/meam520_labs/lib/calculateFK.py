
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

        jointPositions = np.zeros((8,3))
        T0e = np.identity(4)


    #     # DH parameters for robot joints

    #     dh = [
    #         (0, 0, 0.141, 0),
    #         [0, -pi / 2, 0.192, q[0]],
    #         [0, pi / 2, 0, q[1]],
    #         [0.0825, pi / 2, 0.195 + 0.121, q[2] ],
    #         [0.0825, -pi / 2, 0, q[3] + pi ],
    #         [0, -pi / 2, 0.125 + 0.259, q[4]],
    #         [0.088, pi / 2, 0, q[5] - pi ],
    #         [0, 0, 0.051 + 0.159, q[6] - pi / 4],]
        
        

    #     # Calculate the position of each joint

    #     for i in range(8):
    #         T0e = T0e @ self.joint(dh[i])
    #         jointPositions[i] = T0e[0:3, 3]

        
    #     # Your code ends here

    #     return jointPositions, T0e

        
        # DH parameters for robot joints

        dh = [
            (0, 0, 0.141, 0),
            [0, -pi / 2, 0.192, q[0]],
            [0, pi / 2, 0, q[1]],
            [0.0825, pi / 2, 0.195 + 0.121, q[2]],
            [0.0825, pi / 2, 0, q[3] + pi],
            [0, -pi / 2, 0.125 + 0.259, q[4]],
            [0.088, pi / 2, 0, q[5] - pi],
            [0, 0, 0.051 + 0.159, q[6] - pi / 4],]
        

        th = []

        # Calculate the position of each joint

        for i in range(8):
            
            T0e = T0e @ self.joint_transform(dh[i])
            jointPositions[i] = T0e[0:3, 3]
            th.append(T0e)

        jointPositions[2] = (th[2] @ self.joint_transform([0, 0, 0.195, 0]))[0:3, 3]
        jointPositions[4] = (th[4] @ self.joint_transform([0, 0, 0.125, 0]))[0:3, 3]
        jointPositions[5] = (th[5] @ self.joint_transform([0, 0, -0.015, 0]))[0:3, 3]
        jointPositions[6] = (th[6] @ self.joint_transform([0, 0, 0.051, 0]))[0:3, 3]

    
        # Your code ends here

        return jointPositions, T0e
    
    def joint_transform(self, dh):
        a, alpha, d, theta = dh
        cosa = np.cos(alpha)
        cost = np.cos(theta)
        sina = np.sin(alpha)
        sint = np.sin(theta)

        return np.array(
            [
                [cost, -sint * cosa, sint * sina, a * cost],
                [sint, cost * cosa, -cost * sina, a * sint],
                 [0, sina, cosa, d],
                 [0, 0, 0, 1],
             ]
         )

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

        T0e = np.identity(4)
        rotation_axes = np.zeros((3,7))

        dh = [
            (0, 0, 0.141, 0),
            [0, -pi / 2, 0.192, q[0]],
            [0, pi / 2, 0, q[1]],
            [0.0825, pi / 2, 0.195 + 0.121, q[2]],
            [0.0825, pi / 2, 0, q[3] + pi],
            [0, -pi / 2, 0.125 + 0.259, q[4]],
            [0.088, pi / 2, 0, q[5] - pi],
            [0, 0, 0.051 + 0.159, q[6] - pi / 4],]

        for i in range(7):
            T0e = T0e @ self.joint_transform(dh[i])
            current_rotation_matrix= T0e[0:3, 0:3]
            rotation_axes[:, [i]] = current_rotation_matrix @ (np.array([[0,0,1]]).T)
        
            
        return rotation_axes

    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
    #axes = fk.get_axis_of_rotation(q)
    #print("Axes of Rotation:\n", axes)
