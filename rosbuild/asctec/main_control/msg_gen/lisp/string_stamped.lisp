; Auto-generated. Do not edit!


(cl:in-package main_control-msg)


;//! \htmlinclude string_stamped.msg.html

(cl:defclass <string_stamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass string_stamped (<string_stamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <string_stamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'string_stamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name main_control-msg:<string_stamped> is deprecated: use main_control-msg:string_stamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <string_stamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:header-val is deprecated.  Use main_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <string_stamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main_control-msg:data-val is deprecated.  Use main_control-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <string_stamped>) ostream)
  "Serializes a message object of type '<string_stamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <string_stamped>) istream)
  "Deserializes a message object of type '<string_stamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<string_stamped>)))
  "Returns string type for a message object of type '<string_stamped>"
  "main_control/string_stamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'string_stamped)))
  "Returns string type for a message object of type 'string_stamped"
  "main_control/string_stamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<string_stamped>)))
  "Returns md5sum for a message object of type '<string_stamped>"
  "c99a9440709e4d4a9716d55b8270d5e7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'string_stamped)))
  "Returns md5sum for a message object of type 'string_stamped"
  "c99a9440709e4d4a9716d55b8270d5e7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<string_stamped>)))
  "Returns full string definition for message of type '<string_stamped>"
  (cl:format cl:nil "Header header~%string data~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'string_stamped)))
  "Returns full string definition for message of type 'string_stamped"
  (cl:format cl:nil "Header header~%string data~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <string_stamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <string_stamped>))
  "Converts a ROS message object to a list"
  (cl:list 'string_stamped
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
