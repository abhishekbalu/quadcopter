
(cl:in-package :asdf)

(defsystem "rab3D-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Rab3DMsg" :depends-on ("_package_Rab3DMsg"))
    (:file "_package_Rab3DMsg" :depends-on ("_package"))
    (:file "Rab3DObj" :depends-on ("_package_Rab3DObj"))
    (:file "_package_Rab3DObj" :depends-on ("_package"))
  ))