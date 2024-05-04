import cv2
import pdb
import numpy as np
import imageio
import apriltag
import cv2  

def perspective_transform(img, src, targ, targ_size):
    trans_mat = cv2.getPerspectiveTransform(src, targ)
    return cv2.warpPerspective(img, trans_mat, (targ_size)), trans_mat

if __name__ == '__main__':
    img = cv2.imread('./imgs/side_camera_rgb.png')
    pdb.set_trace()

    targ_size = (2000, 1000)
    src_pts = np.array([[1360, 822], [1032, 694], [666, 810], [959, 987]], dtype=np.float32)
    targ_pts = np.array([[1305, 500], [1000, 195], [695, 500], [1000, 805]], dtype=np.float32)

    aerial_img, _ = perspective_transform(img, src_pts, targ_pts, targ_size)   
    imageio.imwrite('./imgs/aerial_view.png', aerial_img) 
# # Load an image
#     image = cv2.imread('path_to_your_image.jpg', cv2.IMREAD_GRAYSCALE)

#     # Create an AprilTag detector object
#     detector = apriltag.Detector()

#     # Detect AprilTags in the image
#     results = detector.detect(image)

#     # Print detection results
#     for r in results:
#         print('Detected:', r.tag_id, 'at', r.center)

    