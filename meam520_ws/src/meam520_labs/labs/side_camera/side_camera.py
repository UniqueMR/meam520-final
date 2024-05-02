from aerial import perspective_transform
from detect_april import is_square, image_to_world, calculate_radius
import numpy as np
import cv2
import apriltag
from PIL import Image
import pdb
import os
import time

class sideCamDetector:
    def __init__(self, trans_mat, targ_size) -> None:
        self.detector = apriltag.Detector()
        self.trans_mat = trans_mat
        self.targ_size = targ_size
        self.detection_stamp = []
        self.transformed_frame = None

    def side_camera_detection(self, img):
        try:
            assert type(img) == type(np.zeros(1)), 'img should be numpy array'
        except AssertionError as error:
            print(f'Error: {error}')
            return

        try:
            assert img.any() != None, 'None values in img'
        except AssertionError as error:
            print(f'Error: {error}')
            return
        
        self.transformed_frame = cv2.warpPerspective(img, self.trans_mat, self.targ_size)
        detections = self.detector.detect(cv2.cvtColor(self.transformed_frame, cv2.COLOR_BGR2GRAY))
        self.detection_stamp = []
        for d in detections:
            if is_square(d, threshold=10):
                self.detection_stamp.append({'position': image_to_world(d.center), 'r': calculate_radius(d.center), 'd': d})
        return self.detection_stamp
    
def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        img = Image.open(os.path.join(folder, filename))
        if img is not None:
            images.append(np.array(img))
    return images

if __name__ == '__main__':
    calibrate_img_path = './imgs/calibrate/calibrate.png'
    calibrate_img = cv2.imread(calibrate_img_path)
    targ_size = (2000, 1000)
    calibrate_src_pts = np.array([[506, 227], [360, 176], [212, 220], [331, 285]], dtype=np.float32)
    calibrate_targ_pts = np.array([[1305, 500], [1000, 195], [695, 500], [1000, 805]], dtype=np.float32)
    calibrate_aerial_img, trans_mat = perspective_transform(calibrate_img, calibrate_src_pts, calibrate_targ_pts, targ_size)
    # pdb.set_trace()
    
    side_detector = sideCamDetector(trans_mat, targ_size)

    frames = load_images_from_folder('./imgs/samples')
    
    for frame in frames:
        time.sleep(0.5)
        detections = side_detector.side_camera_detection(frame)

        for detection in detections:
            # Optionally, draw detections on the frame
            corners = detection['d'].corners
            cv2.polylines(side_detector.transformed_frame, [np.array(corners, np.int32).reshape((-1, 1, 2))], True, (0, 255, 0), 2)
            print('real_world_coord: ', detection['position'])
            print('radius: ', detection['r'])

        # Display the frame
        cv2.imshow('Detection Stream', side_detector.transformed_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
