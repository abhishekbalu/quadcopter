
(cl:in-package :asdf)

(defsystem "ssf_updates-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PositionWithCovarianceStamped" :depends-on ("_package_PositionWithCovarianceStamped"))
    (:file "_package_PositionWithCovarianceStamped" :depends-on ("_package"))
  ))