
(cl:in-package :asdf)

(defsystem "height_quad-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "state" :depends-on ("_package_state"))
    (:file "_package_state" :depends-on ("_package"))
    (:file "EstimateMulti" :depends-on ("_package_EstimateMulti"))
    (:file "_package_EstimateMulti" :depends-on ("_package"))
    (:file "Estimate" :depends-on ("_package_Estimate"))
    (:file "_package_Estimate" :depends-on ("_package"))
    (:file "full_pose" :depends-on ("_package_full_pose"))
    (:file "_package_full_pose" :depends-on ("_package"))
    (:file "EstimateSingle" :depends-on ("_package_EstimateSingle"))
    (:file "_package_EstimateSingle" :depends-on ("_package"))
    (:file "Attitude" :depends-on ("_package_Attitude"))
    (:file "_package_Attitude" :depends-on ("_package"))
  ))