; Auto-generated. Do not edit!


(cl:in-package meam520_labs-msg)


;//! \htmlinclude BlockDetectionArray.msg.html

(cl:defclass <BlockDetectionArray> (roslisp-msg-protocol:ros-message)
  ((detections
    :reader detections
    :initarg :detections
    :type (cl:vector meam520_labs-msg:BlockDetection)
   :initform (cl:make-array 0 :element-type 'meam520_labs-msg:BlockDetection :initial-element (cl:make-instance 'meam520_labs-msg:BlockDetection))))
)

(cl:defclass BlockDetectionArray (<BlockDetectionArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BlockDetectionArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BlockDetectionArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name meam520_labs-msg:<BlockDetectionArray> is deprecated: use meam520_labs-msg:BlockDetectionArray instead.")))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <BlockDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader meam520_labs-msg:detections-val is deprecated.  Use meam520_labs-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BlockDetectionArray>) ostream)
  "Serializes a message object of type '<BlockDetectionArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BlockDetectionArray>) istream)
  "Deserializes a message object of type '<BlockDetectionArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'meam520_labs-msg:BlockDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BlockDetectionArray>)))
  "Returns string type for a message object of type '<BlockDetectionArray>"
  "meam520_labs/BlockDetectionArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BlockDetectionArray)))
  "Returns string type for a message object of type 'BlockDetectionArray"
  "meam520_labs/BlockDetectionArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BlockDetectionArray>)))
  "Returns md5sum for a message object of type '<BlockDetectionArray>"
  "40ad97a01f6e1730f483090b7d388a34")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BlockDetectionArray)))
  "Returns md5sum for a message object of type 'BlockDetectionArray"
  "40ad97a01f6e1730f483090b7d388a34")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BlockDetectionArray>)))
  "Returns full string definition for message of type '<BlockDetectionArray>"
  (cl:format cl:nil "BlockDetection[] detections~%~%================================================================================~%MSG: meam520_labs/BlockDetection~%int16 id~%bool dynamic~%geometry_msgs/PoseStamped pose~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BlockDetectionArray)))
  "Returns full string definition for message of type 'BlockDetectionArray"
  (cl:format cl:nil "BlockDetection[] detections~%~%================================================================================~%MSG: meam520_labs/BlockDetection~%int16 id~%bool dynamic~%geometry_msgs/PoseStamped pose~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BlockDetectionArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BlockDetectionArray>))
  "Converts a ROS message object to a list"
  (cl:list 'BlockDetectionArray
    (cl:cons ':detections (detections msg))
))
