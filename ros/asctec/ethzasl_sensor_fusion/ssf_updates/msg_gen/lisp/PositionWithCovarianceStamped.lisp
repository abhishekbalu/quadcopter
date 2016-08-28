; Auto-generated. Do not edit!


(cl:in-package ssf_updates-msg)


;//! \htmlinclude PositionWithCovarianceStamped.msg.html

(cl:defclass <PositionWithCovarianceStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (covariance
    :reader covariance
    :initarg :covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass PositionWithCovarianceStamped (<PositionWithCovarianceStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PositionWithCovarianceStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PositionWithCovarianceStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ssf_updates-msg:<PositionWithCovarianceStamped> is deprecated: use ssf_updates-msg:PositionWithCovarianceStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PositionWithCovarianceStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ssf_updates-msg:header-val is deprecated.  Use ssf_updates-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <PositionWithCovarianceStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ssf_updates-msg:position-val is deprecated.  Use ssf_updates-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'covariance-val :lambda-list '(m))
(cl:defmethod covariance-val ((m <PositionWithCovarianceStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ssf_updates-msg:covariance-val is deprecated.  Use ssf_updates-msg:covariance instead.")
  (covariance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PositionWithCovarianceStamped>) ostream)
  "Serializes a message object of type '<PositionWithCovarianceStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'covariance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PositionWithCovarianceStamped>) istream)
  "Deserializes a message object of type '<PositionWithCovarianceStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (cl:setf (cl:slot-value msg 'covariance) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'covariance)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PositionWithCovarianceStamped>)))
  "Returns string type for a message object of type '<PositionWithCovarianceStamped>"
  "ssf_updates/PositionWithCovarianceStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PositionWithCovarianceStamped)))
  "Returns string type for a message object of type 'PositionWithCovarianceStamped"
  "ssf_updates/PositionWithCovarianceStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PositionWithCovarianceStamped>)))
  "Returns md5sum for a message object of type '<PositionWithCovarianceStamped>"
  "3128d09129e2da15dcfe14f5a9b5a902")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PositionWithCovarianceStamped)))
  "Returns md5sum for a message object of type 'PositionWithCovarianceStamped"
  "3128d09129e2da15dcfe14f5a9b5a902")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PositionWithCovarianceStamped>)))
  "Returns full string definition for message of type '<PositionWithCovarianceStamped>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point position~%float64[9] covariance~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PositionWithCovarianceStamped)))
  "Returns full string definition for message of type 'PositionWithCovarianceStamped"
  (cl:format cl:nil "Header header~%geometry_msgs/Point position~%float64[9] covariance~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PositionWithCovarianceStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PositionWithCovarianceStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'PositionWithCovarianceStamped
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':covariance (covariance msg))
))
