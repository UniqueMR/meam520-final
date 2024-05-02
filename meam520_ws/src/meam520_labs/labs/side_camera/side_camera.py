from aerial import perspective_transform
from detect_april import is_square, image_to_world
import numpy as np
import cv2
import apriltag
import pdb

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

    # Create an AprilTag detector object
    detector = apriltag.Detector()

    if not cap.isOpened():
        print("Error: Could not open video.")
        exit()

    while cap.isOpened():
        ret, frame = cap.read()
        
        if not ret:
            break

        # Perspective transformation on the frame
        transformed_frame = cv2.warpPerspective(frame, trans_mat, targ_size)

        # Detect AprilTags in the transformed frame
        detections = detector.detect(cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2GRAY))
        for detection in detections:
            if not is_square(detection):
                continue

            # Optionally, draw detections on the frame
            corners = detection.corners
            cv2.polylines(transformed_frame, [np.array(corners, np.int32).reshape((-1, 1, 2))], True, (0, 255, 0), 2)
            print('real_world_coord: ', image_to_world(detection.center))

        # Display the frame
        cv2.imshow('Detection Stream', transformed_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
