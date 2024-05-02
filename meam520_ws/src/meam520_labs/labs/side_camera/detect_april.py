import cv2
import apriltag
import numpy as np
import pdb

def is_square(r, threshold=3.0):
    diag1 = np.sqrt((r.corners[0][0] - r.corners[2][0]) ** 2 + (r.corners[0][1] - r.corners[2][1]) ** 2)
    diag2 = np.sqrt((r.corners[1][0] - r.corners[3][0]) ** 2 + (r.corners[1][1] - r.corners[3][1]) ** 2)
    return np.abs(diag1 - diag2) < threshold

def image_to_world(img_pos, team_id='red'):
    _world_pos = img_pos / 1000
    _world_pos[0] -= 1.0
    _world_pos[1] -= 0.5
    _world_pos[0] = _world_pos[0] + 0.99 if team_id == 'red' else _world_pos[0] - 0.99
    world_pos = np.zeros(2)
    world_pos[0] = _world_pos[1]
    world_pos[1] = _world_pos[0]
    return world_pos

def calculate_radius(img_pos):
    world_pos = img_pos / 1000
    world_pos[0] -= 1.0
    world_pos[1] -= 0.5
    return np.sqrt(world_pos[0] ** 2 + world_pos[1] ** 2)

# Load an image
image = cv2.imread('./imgs/aerial_view.png', cv2.IMREAD_GRAYSCALE)
color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


# Create an AprilTag detector object
detector = apriltag.Detector()

# Detect AprilTags in the image
results = detector.detect(image)

# # Print detection results
# for r in results:
#     print('Detected:', r.tag_id, 'at', r.center)
valid_results = []
for r in results:
    if is_square(r):
        valid_results.append(r)

# Print detection results and draw on the image
for r in valid_results:
    print('Detected:', r.tag_id, 'at', r.center)
    # Draw a rectangle around the AprilTag
    pt1 = (int(r.corners[0][0]), int(r.corners[0][1]))
    pt2 = (int(r.corners[2][0]), int(r.corners[2][1]))
    cv2.rectangle(color_image, pt1, pt2, (0, 255, 0), 2)

    # Draw the center of the tag
    center = tuple(map(int, r.center))
    cv2.circle(color_image, center, 5, (0, 0, 255), -1)

    # Optionally, draw the corners of the tag
    for corner in r.corners:
        corner = tuple(map(int, corner))
        cv2.circle(color_image, corner, 3, (255, 0, 0), -1)

for r in valid_results:
    world_pos = image_to_world(r.center)
    print('positions in world frame: ', world_pos)

# Display the image with detected tags
cv2.imshow('AprilTag Detection', color_image)
cv2.waitKey(0)
cv2.destroyAllWindows()