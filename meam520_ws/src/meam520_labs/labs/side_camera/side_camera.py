import numpy as np
import cv2
import apriltag
import pupil_apriltags
from PIL import Image
import pdb
import os
import time

def perspective_transform(img, src, targ, targ_size):
    trans_mat = cv2.getPerspectiveTransform(src, targ)
    return cv2.warpPerspective(img, trans_mat, (targ_size)), trans_mat

def is_square(r, threshold=3.0):
    diag1 = np.sqrt((r.corners[0][0] - r.corners[2][0]) ** 2 + (r.corners[0][1] - r.corners[2][1]) ** 2)
    diag2 = np.sqrt((r.corners[1][0] - r.corners[3][0]) ** 2 + (r.corners[1][1] - r.corners[3][1]) ** 2)

    edge1 = np.sqrt((r.corners[0][0] - r.corners[1][0]) ** 2 + (r.corners[0][1] - r.corners[1][1]) ** 2)
    edge2 = np.sqrt((r.corners[1][0] - r.corners[2][0]) ** 2 + (r.corners[1][1] - r.corners[2][1]) ** 2) 
    return np.abs(diag1 - diag2) < threshold and np.abs(edge1 - edge2) < threshold

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

class sideCamDetector:
    def __init__(self, trans_mat, targ_size) -> None:
        self.detector = pupil_apriltags.Detector()
        self.trans_mat = trans_mat
        self.targ_size = targ_size
        self.detection_stamp = []
        self.transformed_frame = None

    def side_camera_detection(self, img, team):
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
            if is_square(d, threshold=100.0):
                self.detection_stamp.append({'position': image_to_world(d.center, team_id=team), 'r': calculate_radius(d.center), 'd': d})
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
        detections = side_detector.side_camera_detection(frame, 'red')

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
