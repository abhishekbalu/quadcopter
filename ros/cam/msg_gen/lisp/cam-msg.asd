
(cl:in-package :asdf)

(defsystem "cam-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "imu_acc" :depends-on ("_package_imu_acc"))
    (:file "_package_imu_acc" :depends-on ("_package"))
    (:file "QuadPose" :depends-on ("_package_QuadPose"))
    (:file "_package_QuadPose" :depends-on ("_package"))
    (:file "state_command" :depends-on ("_package_state_command"))
    (:file "_package_state_command" :depends-on ("_package"))
    (:file "full_state" :depends-on ("_package_full_state"))
    (:file "_package_full_state" :depends-on ("_package"))
    (:file "imu" :depends-on ("_package_imu"))
    (:file "_package_imu" :depends-on ("_package"))
    (:file "detections" :depends-on ("_package_detections"))
    (:file "_package_detections" :depends-on ("_package"))
    (:file "QuadPoseList" :depends-on ("_package_QuadPoseList"))
    (:file "_package_QuadPoseList" :depends-on ("_package"))
    (:file "string_stamped" :depends-on ("_package_string_stamped"))
    (:file "_package_string_stamped" :depends-on ("_package"))
    (:file "estimate" :depends-on ("_package_estimate"))
    (:file "_package_estimate" :depends-on ("_package"))
  ))