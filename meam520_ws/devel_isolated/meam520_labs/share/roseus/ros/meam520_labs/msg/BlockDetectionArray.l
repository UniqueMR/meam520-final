;; Auto-generated. Do not edit!


(when (boundp 'meam520_labs::BlockDetectionArray)
  (if (not (find-package "MEAM520_LABS"))
    (make-package "MEAM520_LABS"))
  (shadow 'BlockDetectionArray (find-package "MEAM520_LABS")))
(unless (find-package "MEAM520_LABS::BLOCKDETECTIONARRAY")
  (make-package "MEAM520_LABS::BLOCKDETECTIONARRAY"))

(in-package "ROS")
;;//! \htmlinclude BlockDetectionArray.msg.html


(defclass meam520_labs::BlockDetectionArray
  :super ros::object
  :slots (_detections ))

(defmethod meam520_labs::BlockDetectionArray
  (:init
   (&key
    ((:detections __detections) ())
    )
   (send-super :init)
   (setq _detections __detections)
   self)
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; meam520_labs/BlockDetection[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; meam520_labs/BlockDetection[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; meam520_labs/BlockDetection[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance meam520_labs::BlockDetection :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get meam520_labs::BlockDetectionArray :md5sum-) "40ad97a01f6e1730f483090b7d388a34")
(setf (get meam520_labs::BlockDetectionArray :datatype-) "meam520_labs/BlockDetectionArray")
(setf (get meam520_labs::BlockDetectionArray :definition-)
      "BlockDetection[] detections

================================================================================
MSG: meam520_labs/BlockDetection
int16 id
bool dynamic
geometry_msgs/PoseStamped pose
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :meam520_labs/BlockDetectionArray "40ad97a01f6e1730f483090b7d388a34")


