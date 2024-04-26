import numpy as np

def find_vertical_axis_and_adjust_yaw(transformation_matrix):
    """
    Given a 4x4 transformation matrix, identify which axis of the block is vertically up.
    Then, compute the yaw adjustment needed for the end-effector to align its z-axis down to the block's up-axis.
    
    Args:
    transformation_matrix (numpy.ndarray): A 4x4 transformation matrix from the block to the world.

    Returns:
    tuple: Index of the closest axis (0 for X, 1 for Y, 2 for Z) and required yaw adjustment in radians.
    """
    # Extract the rotation matrix (upper 3x3 of the transformation matrix)
    rotation_matrix = transformation_matrix[:3, :3]
    # Vertical direction in the world frame, assuming block's up is world's down
    vertical_direction = np.array([0, 0, -1])
    # Calculate the dot product between each axis and the vertical direction
    cosines = np.dot(rotation_matrix.T, vertical_direction)
    # Find the axis that is most aligned with the vertical direction
    closest_axis_index = np.argmax(cosines)
    # The required yaw angle to align the end-effector's z-axis with the block's up-axis
    yaw_adjustment = np.arctan2(rotation_matrix[1, closest_axis_index], rotation_matrix[0, closest_axis_index])
    
    return (closest_axis_index, yaw_adjustment)

# Example of a transformation matrix
transformation_matrix = np.array([
    [0.001, 0.999, 0.004, 1],
    [0.002, 0.003, 0.999, 1],
    [-0.999, 0.001, 0.002, 1],
    [0, 0, 0, 1]
])

# Calculate the closest vertical axis and required yaw adjustment
closest_axis_info = find_vertical_axis_and_adjust_yaw(transformation_matrix)
print("Closest axis index:", closest_axis_info[0])
print("Required yaw adjustment (radians):", closest_axis_info[1])
