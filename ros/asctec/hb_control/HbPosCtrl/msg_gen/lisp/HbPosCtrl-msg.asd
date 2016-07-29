
(cl:in-package :asdf)

(defsystem "HbPosCtrl-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ctrl_debug" :depends-on ("_package_ctrl_debug"))
    (:file "_package_ctrl_debug" :depends-on ("_package"))
  ))