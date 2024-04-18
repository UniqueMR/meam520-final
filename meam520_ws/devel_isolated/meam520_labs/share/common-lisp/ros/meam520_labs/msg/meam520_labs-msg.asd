
(cl:in-package :asdf)

(defsystem "meam520_labs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "BlockDetection" :depends-on ("_package_BlockDetection"))
    (:file "_package_BlockDetection" :depends-on ("_package"))
    (:file "BlockDetectionArray" :depends-on ("_package_BlockDetectionArray"))
    (:file "_package_BlockDetectionArray" :depends-on ("_package"))
    (:file "TransformStampedList" :depends-on ("_package_TransformStampedList"))
    (:file "_package_TransformStampedList" :depends-on ("_package"))
  ))