;; Auto-generated. Do not edit!


(when (boundp 'meam520_labs::BlockDetection)
  (if (not (find-package "MEAM520_LABS"))
    (make-package "MEAM520_LABS"))
  (shadow 'BlockDetection (find-package "MEAM520_LABS")))
(unless (find-package "MEAM520_LABS::BLOCKDETECTION")
  (make-package "MEAM520_LABS::BLOCKDETECTION"))

(in-package "ROS")
;;//! \htmlinclude BlockDetection.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass meam520_labs::BlockDetection
  :super ros::object
  :slots (_id _dynamic _pose ))

(defmethod meam520_labs::BlockDetection
  (:init
   (&key
    ((:id __id) 0)
    ((:dynamic __dynamic) nil)
    ((:pose __pose) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _dynamic __dynamic)
   (setq _pose __pose)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:dynamic
   (&optional (__dynamic :null))
   (if (not (eq __dynamic :null)) (setq _dynamic __dynamic)) _dynamic)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; int16 _id
    2
    ;; bool _dynamic
    1
    ;; geometry_msgs/PoseStamped _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _id
       (write-word _id s)
     ;; bool _dynamic
       (if _dynamic (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/PoseStamped _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _id
     (setq _id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _dynamic
     (setq _dynamic (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/PoseStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get meam520_labs::BlockDetection :md5sum-) "f957e4d90fd792d07896c4a471a2f7ef")
(setf (get meam520_labs::BlockDetection :datatype-) "meam520_labs/BlockDetection")
(setf (get meam520_labs::BlockDetection :definition-)
      "int16 id
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



(provide :meam520_labs/BlockDetection "f957e4d90fd792d07896c4a471a2f7ef")


