; Auto-generated. Do not edit!


(cl:in-package main_control-msg)


;//! \htmlinclude state_command.msg.html

(cl:defclass <state_command> (roslisp-msg-protocol:ros-message)
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
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (yaw_rate
    :reader yaw_rate
    :initarg :yaw_rate
    :type cl:float
    :initform 0.0)
   (pos_on
    :reader pos_on
    :initarg :pos_on
    :type cl:fixnum
    :initform 0)
   (yaw_rate_on
    :reader yaw_rate_on
    :initarg :yaw_rate_on
    :type cl:fixnum
    :initform 0)
   (relative_on
    :reader relative_on
    :initarg :relative_on
    :type cl:fixnum
    :initform 0))
)

(cl:defclass state_command (<state_command>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <state_command>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'state_command)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name main_control-msg:<state_command> is deprecated: use main_control-msg:state_command instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <state_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:header-val is deprecated.  Use main_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <state_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:position-val is deprecated.  Use main_control-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <state_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:velocity-val is deprecated.  Use main_control-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <state_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:acceleration-val is deprecated.  Use main_control-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <state_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:yaw-val is deprecated.  Use main_control-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'yaw_rate-val :lambda-list '(m))
(cl:defmethod yaw_rate-val ((m <state_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:yaw_rate-val is deprecated.  Use main_control-msg:yaw_rate instead.")
  (yaw_rate m))

(cl:ensure-generic-function 'pos_on-val :lambda-list '(m))
(cl:defmethod pos_on-val ((m <state_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:pos_on-val is deprecated.  Use main_control-msg:pos_on instead.")
  (pos_on m))

(cl:ensure-generic-function 'yaw_rate_on-val :lambda-list '(m))
(cl:defmethod yaw_rate_on-val ((m <state_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:yaw_rate_on-val is deprecated.  Use main_control-msg:yaw_rate_on instead.")
  (yaw_rate_on m))

(cl:ensure-generic-function 'relative_on-val :lambda-list '(m))
(cl:defmethod relative_on-val ((m <state_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:relative_on-val is deprecated.  Use main_control-msg:relative_on instead.")
  (relative_on m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <state_command>) ostream)
  "Serializes a message object of type '<state_command>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acceleration) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'pos_on)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'yaw_rate_on)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'relative_on)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <state_command>) istream)
  "Deserializes a message object of type '<state_command>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acceleration) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_rate) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pos_on) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'yaw_rate_on) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'relative_on) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<state_command>)))
  "Returns string type for a message object of type '<state_command>"
  "main_control/state_command")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'state_command)))
  "Returns string type for a message object of type 'state_command"
  "main_control/state_command")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<state_command>)))
  "Returns md5sum for a message object of type '<state_command>"
  "91e5adc2b2b2b5ce5be7aa8d7ca96985")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'state_command)))
  "Returns md5sum for a message object of type 'state_command"
  "91e5adc2b2b2b5ce5be7aa8d7ca96985")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<state_command>)))
  "Returns full string definition for message of type '<state_command>"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Vector3       position~%geometry_msgs/Vector3       velocity~%geometry_msgs/Vector3       acceleration~%~%float64 yaw~%float64 yaw_rate~%~%int8 pos_on~%int8 yaw_rate_on~%int8 relative_on~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'state_command)))
  "Returns full string definition for message of type 'state_command"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Vector3       position~%geometry_msgs/Vector3       velocity~%geometry_msgs/Vector3       acceleration~%~%float64 yaw~%float64 yaw_rate~%~%int8 pos_on~%int8 yaw_rate_on~%int8 relative_on~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <state_command>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acceleration))
     8
     8
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <state_command>))
  "Converts a ROS message object to a list"
  (cl:list 'state_command
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':yaw_rate (yaw_rate msg))
    (cl:cons ':pos_on (pos_on msg))
    (cl:cons ':yaw_rate_on (yaw_rate_on msg))
    (cl:cons ':relative_on (relative_on msg))
))
