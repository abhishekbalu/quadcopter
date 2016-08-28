; Auto-generated. Do not edit!


(cl:in-package quad_msgs-msg)


;//! \htmlinclude CamDet.msg.html

(cl:defclass <CamDet> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (blob
    :reader blob
    :initarg :blob
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass CamDet (<CamDet>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CamDet>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CamDet)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quad_msgs-msg:<CamDet> is deprecated: use quad_msgs-msg:CamDet instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CamDet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_msgs-msg:header-val is deprecated.  Use quad_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'blob-val :lambda-list '(m))
(cl:defmethod blob-val ((m <CamDet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_msgs-msg:blob-val is deprecated.  Use quad_msgs-msg:blob instead.")
  (blob m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CamDet>) ostream)
  "Serializes a message object of type '<CamDet>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'blob))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'blob))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CamDet>) istream)
  "Deserializes a message object of type '<CamDet>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'blob) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'blob)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CamDet>)))
  "Returns string type for a message object of type '<CamDet>"
  "quad_msgs/CamDet")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CamDet)))
  "Returns string type for a message object of type 'CamDet"
  "quad_msgs/CamDet")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CamDet>)))
  "Returns md5sum for a message object of type '<CamDet>"
  "ed5d3ef4dd39cff355876f8d615e3b66")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CamDet)))
  "Returns md5sum for a message object of type 'CamDet"
  "ed5d3ef4dd39cff355876f8d615e3b66")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CamDet>)))
  "Returns full string definition for message of type '<CamDet>"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Point[] blob~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CamDet)))
  "Returns full string definition for message of type 'CamDet"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Point[] blob~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CamDet>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'blob) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CamDet>))
  "Converts a ROS message object to a list"
  (cl:list 'CamDet
    (cl:cons ':header (header msg))
    (cl:cons ':blob (blob msg))
))
