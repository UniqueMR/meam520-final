; Auto-generated. Do not edit!


(cl:in-package meam520_labs-msg)


;//! \htmlinclude BlockDetection.msg.html

(cl:defclass <BlockDetection> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (dynamic
    :reader dynamic
    :initarg :dynamic
    :type cl:boolean
    :initform cl:nil)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass BlockDetection (<BlockDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BlockDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BlockDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name meam520_labs-msg:<BlockDetection> is deprecated: use meam520_labs-msg:BlockDetection instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <BlockDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader meam520_labs-msg:id-val is deprecated.  Use meam520_labs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'dynamic-val :lambda-list '(m))
(cl:defmethod dynamic-val ((m <BlockDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader meam520_labs-msg:dynamic-val is deprecated.  Use meam520_labs-msg:dynamic instead.")
  (dynamic m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <BlockDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader meam520_labs-msg:pose-val is deprecated.  Use meam520_labs-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BlockDetection>) ostream)
  "Serializes a message object of type '<BlockDetection>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dynamic) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BlockDetection>) istream)
  "Deserializes a message object of type '<BlockDetection>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:slot-value msg 'dynamic) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BlockDetection>)))
  "Returns string type for a message object of type '<BlockDetection>"
  "meam520_labs/BlockDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BlockDetection)))
  "Returns string type for a message object of type 'BlockDetection"
  "meam520_labs/BlockDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BlockDetection>)))
  "Returns md5sum for a message object of type '<BlockDetection>"
  "f957e4d90fd792d07896c4a471a2f7ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BlockDetection)))
  "Returns md5sum for a message object of type 'BlockDetection"
  "f957e4d90fd792d07896c4a471a2f7ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BlockDetection>)))
  "Returns full string definition for message of type '<BlockDetection>"
  (cl:format cl:nil "int16 id~%bool dynamic~%geometry_msgs/PoseStamped pose~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BlockDetection)))
  "Returns full string definition for message of type 'BlockDetection"
  (cl:format cl:nil "int16 id~%bool dynamic~%geometry_msgs/PoseStamped pose~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BlockDetection>))
  (cl:+ 0
     2
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BlockDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'BlockDetection
    (cl:cons ':id (id msg))
    (cl:cons ':dynamic (dynamic msg))
    (cl:cons ':pose (pose msg))
))
