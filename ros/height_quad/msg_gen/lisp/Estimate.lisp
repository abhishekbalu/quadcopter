; Auto-generated. Do not edit!


(cl:in-package height_quad-msg)


;//! \htmlinclude Estimate.msg.html

(cl:defclass <Estimate> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
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
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (updated
    :reader updated
    :initarg :updated
    :type cl:fixnum
    :initform 0)
   (covariance
    :reader covariance
    :initarg :covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (sensors
    :reader sensors
    :initarg :sensors
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass Estimate (<Estimate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Estimate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Estimate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name height_quad-msg:<Estimate> is deprecated: use height_quad-msg:Estimate instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Estimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader height_quad-msg:name-val is deprecated.  Use height_quad-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <Estimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader height_quad-msg:position-val is deprecated.  Use height_quad-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <Estimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader height_quad-msg:velocity-val is deprecated.  Use height_quad-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'perturbation-val :lambda-list '(m))
(cl:defmethod perturbation-val ((m <Estimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader height_quad-msg:perturbation-val is deprecated.  Use height_quad-msg:perturbation instead.")
  (perturbation m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <Estimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader height_quad-msg:orientation-val is deprecated.  Use height_quad-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'updated-val :lambda-list '(m))
(cl:defmethod updated-val ((m <Estimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader height_quad-msg:updated-val is deprecated.  Use height_quad-msg:updated instead.")
  (updated m))

(cl:ensure-generic-function 'covariance-val :lambda-list '(m))
(cl:defmethod covariance-val ((m <Estimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader height_quad-msg:covariance-val is deprecated.  Use height_quad-msg:covariance instead.")
  (covariance m))

(cl:ensure-generic-function 'sensors-val :lambda-list '(m))
(cl:defmethod sensors-val ((m <Estimate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader height_quad-msg:sensors-val is deprecated.  Use height_quad-msg:sensors instead.")
  (sensors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Estimate>) ostream)
  "Serializes a message object of type '<Estimate>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'name) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'perturbation) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
  (cl:let* ((signed (cl:slot-value msg 'updated)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'covariance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sensors) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Estimate>) istream)
  "Deserializes a message object of type '<Estimate>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'name) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'perturbation) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'updated) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'covariance) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'covariance)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sensors) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Estimate>)))
  "Returns string type for a message object of type '<Estimate>"
  "height_quad/Estimate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Estimate)))
  "Returns string type for a message object of type 'Estimate"
  "height_quad/Estimate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Estimate>)))
  "Returns md5sum for a message object of type '<Estimate>"
  "9dbe5e1fe0043de2a3e9885989b51e45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Estimate)))
  "Returns md5sum for a message object of type 'Estimate"
  "9dbe5e1fe0043de2a3e9885989b51e45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Estimate>)))
  "Returns full string definition for message of type '<Estimate>"
  (cl:format cl:nil "std_msgs/String             name~%geometry_msgs/Vector3       position~%geometry_msgs/Vector3       velocity~%geometry_msgs/Vector3       perturbation~%geometry_msgs/Quaternion    orientation~%int8                        updated~%float64[]                   covariance~%std_msgs/String             sensors~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Estimate)))
  "Returns full string definition for message of type 'Estimate"
  (cl:format cl:nil "std_msgs/String             name~%geometry_msgs/Vector3       position~%geometry_msgs/Vector3       velocity~%geometry_msgs/Vector3       perturbation~%geometry_msgs/Quaternion    orientation~%int8                        updated~%float64[]                   covariance~%std_msgs/String             sensors~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Estimate>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'perturbation))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sensors))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Estimate>))
  "Converts a ROS message object to a list"
  (cl:list 'Estimate
    (cl:cons ':name (name msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':perturbation (perturbation msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':updated (updated msg))
    (cl:cons ':covariance (covariance msg))
    (cl:cons ':sensors (sensors msg))
))
