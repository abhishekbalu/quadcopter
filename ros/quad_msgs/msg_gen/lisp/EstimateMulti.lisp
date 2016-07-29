; Auto-generated. Do not edit!


(cl:in-package quad_msgs-msg)


;//! \htmlinclude EstimateMulti.msg.html

(cl:defclass <EstimateMulti> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (estimates
    :reader estimates
    :initarg :estimates
    :type (cl:vector quad_msgs-msg:Estimate)
   :initform (cl:make-array 0 :element-type 'quad_msgs-msg:Estimate :initial-element (cl:make-instance 'quad_msgs-msg:Estimate)))
   (relative_on
    :reader relative_on
    :initarg :relative_on
    :type cl:fixnum
    :initform 0))
)

(cl:defclass EstimateMulti (<EstimateMulti>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EstimateMulti>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EstimateMulti)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quad_msgs-msg:<EstimateMulti> is deprecated: use quad_msgs-msg:EstimateMulti instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EstimateMulti>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_msgs-msg:header-val is deprecated.  Use quad_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'estimates-val :lambda-list '(m))
(cl:defmethod estimates-val ((m <EstimateMulti>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_msgs-msg:estimates-val is deprecated.  Use quad_msgs-msg:estimates instead.")
  (estimates m))

(cl:ensure-generic-function 'relative_on-val :lambda-list '(m))
(cl:defmethod relative_on-val ((m <EstimateMulti>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_msgs-msg:relative_on-val is deprecated.  Use quad_msgs-msg:relative_on instead.")
  (relative_on m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EstimateMulti>) ostream)
  "Serializes a message object of type '<EstimateMulti>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'estimates))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'estimates))
  (cl:let* ((signed (cl:slot-value msg 'relative_on)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EstimateMulti>) istream)
  "Deserializes a message object of type '<EstimateMulti>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'estimates) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'estimates)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'quad_msgs-msg:Estimate))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'relative_on) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EstimateMulti>)))
  "Returns string type for a message object of type '<EstimateMulti>"
  "quad_msgs/EstimateMulti")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EstimateMulti)))
  "Returns string type for a message object of type 'EstimateMulti"
  "quad_msgs/EstimateMulti")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EstimateMulti>)))
  "Returns md5sum for a message object of type '<EstimateMulti>"
  "ba869d9da0e26b7e01d4a406b2b4ae60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EstimateMulti)))
  "Returns md5sum for a message object of type 'EstimateMulti"
  "ba869d9da0e26b7e01d4a406b2b4ae60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EstimateMulti>)))
  "Returns full string definition for message of type '<EstimateMulti>"
  (cl:format cl:nil "Header header~%Estimate[] estimates~%int8 relative_on~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: quad_msgs/Estimate~%std_msgs/String             name~%geometry_msgs/Vector3       position~%geometry_msgs/Vector3       velocity~%geometry_msgs/Vector3       perturbation~%geometry_msgs/Quaternion    orientation~%int8                        updated~%float64[]                   covariance~%std_msgs/String             sensors~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EstimateMulti)))
  "Returns full string definition for message of type 'EstimateMulti"
  (cl:format cl:nil "Header header~%Estimate[] estimates~%int8 relative_on~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: quad_msgs/Estimate~%std_msgs/String             name~%geometry_msgs/Vector3       position~%geometry_msgs/Vector3       velocity~%geometry_msgs/Vector3       perturbation~%geometry_msgs/Quaternion    orientation~%int8                        updated~%float64[]                   covariance~%std_msgs/String             sensors~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EstimateMulti>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'estimates) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EstimateMulti>))
  "Converts a ROS message object to a list"
  (cl:list 'EstimateMulti
    (cl:cons ':header (header msg))
    (cl:cons ':estimates (estimates msg))
    (cl:cons ':relative_on (relative_on msg))
))
