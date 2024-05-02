from aerial import perspective_transform
from detect_april import is_square, image_to_world, calculate_radius
import numpy as np
import cv2
import apriltag
import pdb

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
            if is_square(d):
                self.detection_stamp.append({'position': image_to_world(d.center), 'r': calculate_radius(d.center), 'd': d})
        return self.detection_stamp
    

if __name__ == '__main__':
    calibrate_img_path = './imgs/calibration_image.png'
    calibrate_img = cv2.imread(calibrate_img_path)
    targ_size = (2000, 1000)
    calibrate_src_pts = np.array([[1170, 613], [885, 454], [392, 561], [680, 783]], dtype=np.float32)
    calibrate_targ_pts = np.array([[1305, 500], [1000, 195], [695, 500], [1000, 805]], dtype=np.float32)
    calibrate_aerial_img, trans_mat = perspective_transform(calibrate_img, calibrate_src_pts, calibrate_targ_pts, targ_size)
    # pdb.set_trace()

    # Video capture
    video_path = './imgs/side_camera_stream.mov'
    cap = cv2.VideoCapture(video_path)
    
    side_detector = sideCamDetector(trans_mat, targ_size)

    if not cap.isOpened():
        print("Error: Could not open video.")
        exit()

    while cap.isOpened():
        ret, frame = cap.read()
        
        if not ret:
            break

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

    cap.release()
    cv2.destroyAllWindows()
