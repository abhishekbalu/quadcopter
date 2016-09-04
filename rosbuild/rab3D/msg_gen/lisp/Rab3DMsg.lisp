; Auto-generated. Do not edit!


(cl:in-package rab3D-msg)


;//! \htmlinclude Rab3DMsg.msg.html

(cl:defclass <Rab3DMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (spi1_miss
    :reader spi1_miss
    :initarg :spi1_miss
    :type cl:fixnum
    :initform 0)
   (spi2_miss
    :reader spi2_miss
    :initarg :spi2_miss
    :type cl:fixnum
    :initform 0)
   (uart_miss
    :reader uart_miss
    :initarg :uart_miss
    :type cl:fixnum
    :initform 0)
   (objects
    :reader objects
    :initarg :objects
    :type (cl:vector rab3D-msg:Rab3DObj)
   :initform (cl:make-array 0 :element-type 'rab3D-msg:Rab3DObj :initial-element (cl:make-instance 'rab3D-msg:Rab3DObj))))
)

(cl:defclass Rab3DMsg (<Rab3DMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Rab3DMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Rab3DMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rab3D-msg:<Rab3DMsg> is deprecated: use rab3D-msg:Rab3DMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Rab3DMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:header-val is deprecated.  Use rab3D-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'spi1_miss-val :lambda-list '(m))
(cl:defmethod spi1_miss-val ((m <Rab3DMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:spi1_miss-val is deprecated.  Use rab3D-msg:spi1_miss instead.")
  (spi1_miss m))

(cl:ensure-generic-function 'spi2_miss-val :lambda-list '(m))
(cl:defmethod spi2_miss-val ((m <Rab3DMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:spi2_miss-val is deprecated.  Use rab3D-msg:spi2_miss instead.")
  (spi2_miss m))

(cl:ensure-generic-function 'uart_miss-val :lambda-list '(m))
(cl:defmethod uart_miss-val ((m <Rab3DMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:uart_miss-val is deprecated.  Use rab3D-msg:uart_miss instead.")
  (uart_miss m))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <Rab3DMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:objects-val is deprecated.  Use rab3D-msg:objects instead.")
  (objects m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Rab3DMsg>) ostream)
  "Serializes a message object of type '<Rab3DMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'spi1_miss)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'spi2_miss)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'uart_miss)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'objects))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Rab3DMsg>) istream)
  "Deserializes a message object of type '<Rab3DMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'spi1_miss) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'spi2_miss) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'uart_miss) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'rab3D-msg:Rab3DObj))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Rab3DMsg>)))
  "Returns string type for a message object of type '<Rab3DMsg>"
  "rab3D/Rab3DMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Rab3DMsg)))
  "Returns string type for a message object of type 'Rab3DMsg"
  "rab3D/Rab3DMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Rab3DMsg>)))
  "Returns md5sum for a message object of type '<Rab3DMsg>"
  "c50719e217d7b85248e8bb552deaaa75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Rab3DMsg)))
  "Returns md5sum for a message object of type 'Rab3DMsg"
  "c50719e217d7b85248e8bb552deaaa75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Rab3DMsg>)))
  "Returns full string definition for message of type '<Rab3DMsg>"
  (cl:format cl:nil "Header header~%~%int8 spi1_miss~%int8 spi2_miss~%int8 uart_miss	~%Rab3DObj[] objects~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: rab3D/Rab3DObj~%int16 slot~%int32 code~%int16 adc1~%int16 adc2~%int16 adc3~%int16 adc4~%int16 adc5~%int16 adc6~%int16 adc7~%int16 adc8~%int16 adc9~%int16 adc10~%int16 adc11~%int16 adc12~%int16 adc13~%int16 adc14~%int16 adc15~%int16 adc16~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Rab3DMsg)))
  "Returns full string definition for message of type 'Rab3DMsg"
  (cl:format cl:nil "Header header~%~%int8 spi1_miss~%int8 spi2_miss~%int8 uart_miss	~%Rab3DObj[] objects~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: rab3D/Rab3DObj~%int16 slot~%int32 code~%int16 adc1~%int16 adc2~%int16 adc3~%int16 adc4~%int16 adc5~%int16 adc6~%int16 adc7~%int16 adc8~%int16 adc9~%int16 adc10~%int16 adc11~%int16 adc12~%int16 adc13~%int16 adc14~%int16 adc15~%int16 adc16~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Rab3DMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Rab3DMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'Rab3DMsg
    (cl:cons ':header (header msg))
    (cl:cons ':spi1_miss (spi1_miss msg))
    (cl:cons ':spi2_miss (spi2_miss msg))
    (cl:cons ':uart_miss (uart_miss msg))
    (cl:cons ':objects (objects msg))
))
