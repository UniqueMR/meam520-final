import cv2

video_path = './imgs/side_camera_stream.mov'
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

ret, first_frame = cap.read()

if ret:
    cv2.imwrite('./imgs/calibration_image.png', first_frame)  # Save the first frame as an image file if needed
    calibrate_img = first_frame  # Use this frame for calibration
else:
    print("Error: Could not read the first frame.")

cap.release()
