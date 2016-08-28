
(cl:in-package :asdf)

(defsystem "ssf_core-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DoubleArrayStamped" :depends-on ("_package_DoubleArrayStamped"))
    (:file "_package_DoubleArrayStamped" :depends-on ("_package"))
    (:file "ext_ekf" :depends-on ("_package_ext_ekf"))
    (:file "_package_ext_ekf" :depends-on ("_package"))
    (:file "ext_imu" :depends-on ("_package_ext_imu"))
    (:file "_package_ext_imu" :depends-on ("_package"))
    (:file "ext_state" :depends-on ("_package_ext_state"))
    (:file "_package_ext_state" :depends-on ("_package"))
  ))