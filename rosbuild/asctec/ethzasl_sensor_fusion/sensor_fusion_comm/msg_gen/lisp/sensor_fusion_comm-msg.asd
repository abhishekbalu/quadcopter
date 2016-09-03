
(cl:in-package :asdf)

(defsystem "sensor_fusion_comm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DoubleArrayStamped" :depends-on ("_package_DoubleArrayStamped"))
    (:file "_package_DoubleArrayStamped" :depends-on ("_package"))
    (:file "ExtState" :depends-on ("_package_ExtState"))
    (:file "_package_ExtState" :depends-on ("_package"))
    (:file "ExtEkf" :depends-on ("_package_ExtEkf"))
    (:file "_package_ExtEkf" :depends-on ("_package"))
  ))