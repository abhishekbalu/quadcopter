; Auto-generated. Do not edit!


(cl:in-package main_control-msg)


;//! \htmlinclude estimate.msg.html

(cl:defclass <estimate> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (perturbation
    :reader perturbation
    :initarg :perturbation
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (orientation
    :reader orientation
    :initarg :orientation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion)))
)

(cl:defclass estimate (<estimate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <estimate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'estimate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name main_control-msg:<estimate> is deprecated: use main_control-msg:estimate instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <estimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:header-val is deprecated.  Use main_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <estimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:position-val is deprecated.  Use main_control-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <estimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:velocity-val is deprecated.  Use main_control-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'perturbation-val :lambda-list '(m))
(cl:defmethod perturbation-val ((m <estimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:perturbation-val is deprecated.  Use main_control-msg:perturbation instead.")
  (perturbation m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <estimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:orientation-val is deprecated.  Use main_control-msg:orientation instead.")
  (orientation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <estimate>) ostream)
  "Serializes a message object of type '<estimate>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'perturbation) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <estimate>) istream)
  "Deserializes a message object of type '<estimate>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'perturbation) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<estimate>)))
  "Returns string type for a message object of type '<estimate>"
  "main_control/estimate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'estimate)))
  "Returns string type for a message object of type 'estimate"
  "main_control/estimate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<estimate>)))
  "Returns md5sum for a message object of type '<estimate>"
  "04dc4417346081189209271e36ab818d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'estimate)))
  "Returns md5sum for a message object of type 'estimate"
  "04dc4417346081189209271e36ab818d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<estimate>)))
  "Returns full string definition for message of type '<estimate>"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Vector3       position~%geometry_msgs/Vector3       velocity~%geometry_msgs/Vector3       perturbation~%geometry_msgs/Quaternion    orientation~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'estimate)))
  "Returns full string definition for message of type 'estimate"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Vector3       position~%geometry_msgs/Vector3       velocity~%geometry_msgs/Vector3       perturbation~%geometry_msgs/Quaternion    orientation~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <estimate>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'perturbation))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <estimate>))
  "Converts a ROS message object to a list"
  (cl:list 'estimate
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':perturbation (perturbation msg))
    (cl:cons ':orientation (orientation msg))
))
