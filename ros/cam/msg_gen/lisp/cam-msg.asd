
(cl:in-package :asdf)

(defsystem "cam-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "detections" :depends-on ("_package_detections"))
    (:file "_package_detections" :depends-on ("_package"))
    (:file "QuadPoseList" :depends-on ("_package_QuadPoseList"))
    (:file "_package_QuadPoseList" :depends-on ("_package"))
    (:file "QuadPose" :depends-on ("_package_QuadPose"))
    (:file "_package_QuadPose" :depends-on ("_package"))
  ))