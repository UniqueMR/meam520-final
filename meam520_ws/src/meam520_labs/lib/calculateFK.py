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
        
        x = []
        t = [
            [0, 0, 0.141, 0],
            [0, -pi / 2, 0.192, q[0]],
            [0, pi / 2, 0, q[1]],
            [0.0825, pi / 2, 0.195 + 0.121, q[2]],
            [0.0825, pi / 2, 0, q[3] + pi],
            [0, -pi / 2, 0.125 + 0.259, q[4]],
            [0.088, pi / 2, 0, q[5] - pi],
            [0, 0, 0.051 + 0.159, q[6] - pi / 4],
        ]
        
        i = 0
        while i < 8:
            T0e = np.dot(T0e, self.joint_cal(t[i]))
            x.append(T0e)
            jointPositions[i] = T0e[0:3, 3]
            i += 1

        #jointPositions[0] = np.dot(np.identity(4), self.joint_cal([0, 0, 0.141, 0]))[0:3, 3]
        #jointPositions[0] = [0, 0, 0.141]       
        jointPositions[2] = np.dot(x[2], self.joint_cal([0, 0, 0.195, 0]))[0:3, 3]
        jointPositions[4] = np.dot(x[4], self.joint_cal([0, 0, 0.125, 0]))[0:3, 3]
        jointPositions[5] = np.dot(x[5], self.joint_cal([0, 0, -0.015, 0]))[0:3, 3]
        jointPositions[6] = np.dot(x[6], self.joint_cal([0, 0, 0.051, 0]))[0:3, 3]

        # Your code ends here
        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1
    def joint_cal(self, t: list):
    
    	a, alpha, d, theta = t
    	sint = np.sin(theta)
    	sina = np.sin(alpha)
    	cost = np.cos(theta)
    	cosa = np.cos(alpha)
    	
    	res = np.array([[cost, -sint * cosa, sint * sina, a * cost],
    	[sint, cost * cosa, -cost * sina, a * sint],
    	[0, sina, cosa, d],
    	[0, 0, 0, 1],])
    	
    	return res
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
        t = [
            [0, 0, 0.141, 0],
            [0, -pi / 2, 0.192, q[0]],
            [0, pi / 2, 0, q[1]],
            [0.0825, pi / 2, 0.195 + 0.121, q[2]],
            [0.0825, pi / 2, 0, q[3] + pi],
            [0, -pi / 2, 0.125 + 0.259, q[4]],
            [0.088, pi / 2, 0, q[5] - pi],
            [0, 0, 0.051 + 0.159, q[6] - pi / 4],
            ]
        
        z_rotation = np.zeros((3,7))
        T0e = np.identity(4)
        
        i = 0
        while i < 7:
            T0e = np.dot(T0e, self.joint_cal(t[i]))
            z_rotation[:, i] = T0e[0:3, 0:3] @ np.array([0, 0, 1]).T
            i += 1

        return z_rotation


    
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
    #q = np.array([ pi/2, 0,  pi/4, -pi/2, -pi/2, pi/2,    0 ])
    #q = np.array([ 0, 0,  -pi/2, -pi/4, -pi/2, pi,    pi/4 ])

    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
