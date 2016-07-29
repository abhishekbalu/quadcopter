; Auto-generated. Do not edit!


(cl:in-package rab3D-msg)


;//! \htmlinclude Rab3DObj.msg.html

(cl:defclass <Rab3DObj> (roslisp-msg-protocol:ros-message)
  ((slot
    :reader slot
    :initarg :slot
    :type cl:fixnum
    :initform 0)
   (code
    :reader code
    :initarg :code
    :type cl:integer
    :initform 0)
   (adc1
    :reader adc1
    :initarg :adc1
    :type cl:fixnum
    :initform 0)
   (adc2
    :reader adc2
    :initarg :adc2
    :type cl:fixnum
    :initform 0)
   (adc3
    :reader adc3
    :initarg :adc3
    :type cl:fixnum
    :initform 0)
   (adc4
    :reader adc4
    :initarg :adc4
    :type cl:fixnum
    :initform 0)
   (adc5
    :reader adc5
    :initarg :adc5
    :type cl:fixnum
    :initform 0)
   (adc6
    :reader adc6
    :initarg :adc6
    :type cl:fixnum
    :initform 0)
   (adc7
    :reader adc7
    :initarg :adc7
    :type cl:fixnum
    :initform 0)
   (adc8
    :reader adc8
    :initarg :adc8
    :type cl:fixnum
    :initform 0)
   (adc9
    :reader adc9
    :initarg :adc9
    :type cl:fixnum
    :initform 0)
   (adc10
    :reader adc10
    :initarg :adc10
    :type cl:fixnum
    :initform 0)
   (adc11
    :reader adc11
    :initarg :adc11
    :type cl:fixnum
    :initform 0)
   (adc12
    :reader adc12
    :initarg :adc12
    :type cl:fixnum
    :initform 0)
   (adc13
    :reader adc13
    :initarg :adc13
    :type cl:fixnum
    :initform 0)
   (adc14
    :reader adc14
    :initarg :adc14
    :type cl:fixnum
    :initform 0)
   (adc15
    :reader adc15
    :initarg :adc15
    :type cl:fixnum
    :initform 0)
   (adc16
    :reader adc16
    :initarg :adc16
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Rab3DObj (<Rab3DObj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Rab3DObj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Rab3DObj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rab3D-msg:<Rab3DObj> is deprecated: use rab3D-msg:Rab3DObj instead.")))

(cl:ensure-generic-function 'slot-val :lambda-list '(m))
(cl:defmethod slot-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:slot-val is deprecated.  Use rab3D-msg:slot instead.")
  (slot m))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:code-val is deprecated.  Use rab3D-msg:code instead.")
  (code m))

(cl:ensure-generic-function 'adc1-val :lambda-list '(m))
(cl:defmethod adc1-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc1-val is deprecated.  Use rab3D-msg:adc1 instead.")
  (adc1 m))

(cl:ensure-generic-function 'adc2-val :lambda-list '(m))
(cl:defmethod adc2-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc2-val is deprecated.  Use rab3D-msg:adc2 instead.")
  (adc2 m))

(cl:ensure-generic-function 'adc3-val :lambda-list '(m))
(cl:defmethod adc3-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc3-val is deprecated.  Use rab3D-msg:adc3 instead.")
  (adc3 m))

(cl:ensure-generic-function 'adc4-val :lambda-list '(m))
(cl:defmethod adc4-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc4-val is deprecated.  Use rab3D-msg:adc4 instead.")
  (adc4 m))

(cl:ensure-generic-function 'adc5-val :lambda-list '(m))
(cl:defmethod adc5-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc5-val is deprecated.  Use rab3D-msg:adc5 instead.")
  (adc5 m))

(cl:ensure-generic-function 'adc6-val :lambda-list '(m))
(cl:defmethod adc6-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc6-val is deprecated.  Use rab3D-msg:adc6 instead.")
  (adc6 m))

(cl:ensure-generic-function 'adc7-val :lambda-list '(m))
(cl:defmethod adc7-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc7-val is deprecated.  Use rab3D-msg:adc7 instead.")
  (adc7 m))

(cl:ensure-generic-function 'adc8-val :lambda-list '(m))
(cl:defmethod adc8-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc8-val is deprecated.  Use rab3D-msg:adc8 instead.")
  (adc8 m))

(cl:ensure-generic-function 'adc9-val :lambda-list '(m))
(cl:defmethod adc9-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc9-val is deprecated.  Use rab3D-msg:adc9 instead.")
  (adc9 m))

(cl:ensure-generic-function 'adc10-val :lambda-list '(m))
(cl:defmethod adc10-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc10-val is deprecated.  Use rab3D-msg:adc10 instead.")
  (adc10 m))

(cl:ensure-generic-function 'adc11-val :lambda-list '(m))
(cl:defmethod adc11-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc11-val is deprecated.  Use rab3D-msg:adc11 instead.")
  (adc11 m))

(cl:ensure-generic-function 'adc12-val :lambda-list '(m))
(cl:defmethod adc12-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc12-val is deprecated.  Use rab3D-msg:adc12 instead.")
  (adc12 m))

(cl:ensure-generic-function 'adc13-val :lambda-list '(m))
(cl:defmethod adc13-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc13-val is deprecated.  Use rab3D-msg:adc13 instead.")
  (adc13 m))

(cl:ensure-generic-function 'adc14-val :lambda-list '(m))
(cl:defmethod adc14-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc14-val is deprecated.  Use rab3D-msg:adc14 instead.")
  (adc14 m))

(cl:ensure-generic-function 'adc15-val :lambda-list '(m))
(cl:defmethod adc15-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc15-val is deprecated.  Use rab3D-msg:adc15 instead.")
  (adc15 m))

(cl:ensure-generic-function 'adc16-val :lambda-list '(m))
(cl:defmethod adc16-val ((m <Rab3DObj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rab3D-msg:adc16-val is deprecated.  Use rab3D-msg:adc16 instead.")
  (adc16 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Rab3DObj>) ostream)
  "Serializes a message object of type '<Rab3DObj>"
  (cl:let* ((signed (cl:slot-value msg 'slot)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc5)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc6)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc7)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc8)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc9)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc10)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc11)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc12)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc13)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc14)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc15)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'adc16)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Rab3DObj>) istream)
  "Deserializes a message object of type '<Rab3DObj>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'slot) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'code) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc1) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc2) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc3) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc4) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc5) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc6) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc7) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc8) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc9) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc10) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc11) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc12) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc13) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc14) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc15) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'adc16) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Rab3DObj>)))
  "Returns string type for a message object of type '<Rab3DObj>"
  "rab3D/Rab3DObj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Rab3DObj)))
  "Returns string type for a message object of type 'Rab3DObj"
  "rab3D/Rab3DObj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Rab3DObj>)))
  "Returns md5sum for a message object of type '<Rab3DObj>"
  "24f96da6018bfad44f2cacae52c1832b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Rab3DObj)))
  "Returns md5sum for a message object of type 'Rab3DObj"
  "24f96da6018bfad44f2cacae52c1832b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Rab3DObj>)))
  "Returns full string definition for message of type '<Rab3DObj>"
  (cl:format cl:nil "int16 slot~%int32 code~%int16 adc1~%int16 adc2~%int16 adc3~%int16 adc4~%int16 adc5~%int16 adc6~%int16 adc7~%int16 adc8~%int16 adc9~%int16 adc10~%int16 adc11~%int16 adc12~%int16 adc13~%int16 adc14~%int16 adc15~%int16 adc16~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Rab3DObj)))
  "Returns full string definition for message of type 'Rab3DObj"
  (cl:format cl:nil "int16 slot~%int32 code~%int16 adc1~%int16 adc2~%int16 adc3~%int16 adc4~%int16 adc5~%int16 adc6~%int16 adc7~%int16 adc8~%int16 adc9~%int16 adc10~%int16 adc11~%int16 adc12~%int16 adc13~%int16 adc14~%int16 adc15~%int16 adc16~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Rab3DObj>))
  (cl:+ 0
     2
     4
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Rab3DObj>))
  "Converts a ROS message object to a list"
  (cl:list 'Rab3DObj
    (cl:cons ':slot (slot msg))
    (cl:cons ':code (code msg))
    (cl:cons ':adc1 (adc1 msg))
    (cl:cons ':adc2 (adc2 msg))
    (cl:cons ':adc3 (adc3 msg))
    (cl:cons ':adc4 (adc4 msg))
    (cl:cons ':adc5 (adc5 msg))
    (cl:cons ':adc6 (adc6 msg))
    (cl:cons ':adc7 (adc7 msg))
    (cl:cons ':adc8 (adc8 msg))
    (cl:cons ':adc9 (adc9 msg))
    (cl:cons ':adc10 (adc10 msg))
    (cl:cons ':adc11 (adc11 msg))
    (cl:cons ':adc12 (adc12 msg))
    (cl:cons ':adc13 (adc13 msg))
    (cl:cons ':adc14 (adc14 msg))
    (cl:cons ':adc15 (adc15 msg))
    (cl:cons ':adc16 (adc16 msg))
))
