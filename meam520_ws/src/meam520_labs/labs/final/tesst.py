import numpy as np

def find_vertical_axis_and_adjust_yaw(transformation_matrix):
    # Extract the rotation matrix (upper 3x3 of the transformation matrix)
    rotation_matrix = transformation_matrix[:3, :3]
    # Vertical direction in the world frame, assuming block's up is world's down
    vertical_direction = np.array([0, 0, -1])
    # Calculate the dot product between each axis and the vertical direction
    cosines = np.dot(rotation_matrix.T, vertical_direction)
    # Find the axis that is most aligned with the vertical direction
    closest_axis_index = np.argmax(np.abs(cosines))
    # The required yaw angle to align the end-effector's z-axis with the block's up-axis
    yaw_adjustment = np.arctan2(rotation_matrix[1, closest_axis_index], rotation_matrix[0, closest_axis_index])
    
    return (closest_axis_index, yaw_adjustment)

def matrix_to_yaw_pitch_roll(matrix):
    sy = np.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])

    singular = sy < 1e-6

    if not singular:
        yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
        pitch = np.arctan2(-matrix[2, 0], sy)
        roll = np.arctan2(matrix[2, 1], matrix[2, 2])
    else:
        yaw = np.arctan2(-matrix[1, 2], matrix[1, 1])
        pitch = np.arctan2(-matrix[2, 0], sy)
        roll = 0
    angle_set = [pitch,yaw,roll] # x,y,z
    return angle_set

# Example of a transformation matrix
