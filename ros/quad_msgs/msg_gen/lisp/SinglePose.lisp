; Auto-generated. Do not edit!


(cl:in-package quad_msgs-msg)


;//! \htmlinclude SinglePose.msg.html

(cl:defclass <SinglePose> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (orientation
    :reader orientation
    :initarg :orientation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (pose_updated
    :reader pose_updated
    :initarg :pose_updated
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SinglePose (<SinglePose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SinglePose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SinglePose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quad_msgs-msg:<SinglePose> is deprecated: use quad_msgs-msg:SinglePose instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <SinglePose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_msgs-msg:name-val is deprecated.  Use quad_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <SinglePose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_msgs-msg:position-val is deprecated.  Use quad_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <SinglePose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_msgs-msg:orientation-val is deprecated.  Use quad_msgs-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'pose_updated-val :lambda-list '(m))
(cl:defmethod pose_updated-val ((m <SinglePose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quad_msgs-msg:pose_updated-val is deprecated.  Use quad_msgs-msg:pose_updated instead.")
  (pose_updated m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SinglePose>) ostream)
  "Serializes a message object of type '<SinglePose>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
  (cl:let* ((signed (cl:slot-value msg 'pose_updated)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SinglePose>) istream)
  "Deserializes a message object of type '<SinglePose>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pose_updated) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SinglePose>)))
  "Returns string type for a message object of type '<SinglePose>"
  "quad_msgs/SinglePose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SinglePose)))
  "Returns string type for a message object of type 'SinglePose"
  "quad_msgs/SinglePose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SinglePose>)))
  "Returns md5sum for a message object of type '<SinglePose>"
  "180c6571038f982cd09da891297d01b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SinglePose)))
  "Returns md5sum for a message object of type 'SinglePose"
  "180c6571038f982cd09da891297d01b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SinglePose>)))
  "Returns full string definition for message of type '<SinglePose>"
  (cl:format cl:nil "string name~%geometry_msgs/Point position~%geometry_msgs/Quaternion orientation~%int8 pose_updated~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SinglePose)))
  "Returns full string definition for message of type 'SinglePose"
  (cl:format cl:nil "string name~%geometry_msgs/Point position~%geometry_msgs/Quaternion orientation~%int8 pose_updated~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SinglePose>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SinglePose>))
  "Converts a ROS message object to a list"
  (cl:list 'SinglePose
    (cl:cons ':name (name msg))
    (cl:cons ':position (position msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':pose_updated (pose_updated msg))
))
