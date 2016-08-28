; Auto-generated. Do not edit!


(cl:in-package quad_msgs-msg)


;//! \htmlinclude TargetsWithCov.msg.html

(cl:defclass <TargetsWithCov> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (targets
    :reader targets
    :initarg :targets
    :type (cl:vector quad_msgs-msg:StateWithCov)
   :initform (cl:make-array 0 :element-type 'quad_msgs-msg:StateWithCov :initial-element (cl:make-instance 'quad_msgs-msg:StateWithCov))))
)

(cl:defclass TargetsWithCov (<TargetsWithCov>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetsWithCov>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetsWithCov)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quad_msgs-msg:<TargetsWithCov> is deprecated: use quad_msgs-msg:TargetsWithCov instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TargetsWithCov>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_msgs-msg:header-val is deprecated.  Use quad_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'targets-val :lambda-list '(m))
(cl:defmethod targets-val ((m <TargetsWithCov>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_msgs-msg:targets-val is deprecated.  Use quad_msgs-msg:targets instead.")
  (targets m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetsWithCov>) ostream)
  "Serializes a message object of type '<TargetsWithCov>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'targets))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'targets))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetsWithCov>) istream)
  "Deserializes a message object of type '<TargetsWithCov>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'targets) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'targets)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'quad_msgs-msg:StateWithCov))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetsWithCov>)))
  "Returns string type for a message object of type '<TargetsWithCov>"
  "quad_msgs/TargetsWithCov")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetsWithCov)))
  "Returns string type for a message object of type 'TargetsWithCov"
  "quad_msgs/TargetsWithCov")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetsWithCov>)))
  "Returns md5sum for a message object of type '<TargetsWithCov>"
  "ce9af668335a6638d3822f57471a8465")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetsWithCov)))
  "Returns md5sum for a message object of type 'TargetsWithCov"
  "ce9af668335a6638d3822f57471a8465")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetsWithCov>)))
  "Returns full string definition for message of type '<TargetsWithCov>"
  (cl:format cl:nil "Header header~%StateWithCov[] targets~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: quad_msgs/StateWithCov~%uint8 number_states~%float64[] states~%float64[] covariance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetsWithCov)))
  "Returns full string definition for message of type 'TargetsWithCov"
  (cl:format cl:nil "Header header~%StateWithCov[] targets~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: quad_msgs/StateWithCov~%uint8 number_states~%float64[] states~%float64[] covariance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetsWithCov>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'targets) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetsWithCov>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetsWithCov
    (cl:cons ':header (header msg))
    (cl:cons ':targets (targets msg))
))
