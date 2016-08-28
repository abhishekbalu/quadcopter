; Auto-generated. Do not edit!


(cl:in-package sensor_fusion_comm-msg)


;//! \htmlinclude ExtState.msg.html

(cl:defclass <ExtState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass ExtState (<ExtState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExtState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExtState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_fusion_comm-msg:<ExtState> is deprecated: use sensor_fusion_comm-msg:ExtState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ExtState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_fusion_comm-msg:header-val is deprecated.  Use sensor_fusion_comm-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <ExtState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_fusion_comm-msg:pose-val is deprecated.  Use sensor_fusion_comm-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <ExtState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_fusion_comm-msg:velocity-val is deprecated.  Use sensor_fusion_comm-msg:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExtState>) ostream)
  "Serializes a message object of type '<ExtState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExtState>) istream)
  "Deserializes a message object of type '<ExtState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExtState>)))
  "Returns string type for a message object of type '<ExtState>"
  "sensor_fusion_comm/ExtState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExtState)))
  "Returns string type for a message object of type 'ExtState"
  "sensor_fusion_comm/ExtState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExtState>)))
  "Returns md5sum for a message object of type '<ExtState>"
  "e034f695cee32efa98d5cb7e960000e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExtState)))
  "Returns md5sum for a message object of type 'ExtState"
  "e034f695cee32efa98d5cb7e960000e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExtState>)))
  "Returns full string definition for message of type '<ExtState>"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose          pose~%geometry_msgs/Vector3       velocity~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExtState)))
  "Returns full string definition for message of type 'ExtState"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose          pose~%geometry_msgs/Vector3       velocity~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExtState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExtState>))
  "Converts a ROS message object to a list"
  (cl:list 'ExtState
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':velocity (velocity msg))
))
