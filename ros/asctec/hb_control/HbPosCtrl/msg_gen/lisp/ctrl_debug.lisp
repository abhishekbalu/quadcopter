; Auto-generated. Do not edit!


(cl:in-package HbPosCtrl-msg)


;//! \htmlinclude ctrl_debug.msg.html

(cl:defclass <ctrl_debug> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (epos
    :reader epos
    :initarg :epos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (evel
    :reader evel
    :initarg :evel
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (fdes
    :reader fdes
    :initarg :fdes
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (acc
    :reader acc
    :initarg :acc
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (accb
    :reader accb
    :initarg :accb
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (accref
    :reader accref
    :initarg :accref
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (velref
    :reader velref
    :initarg :velref
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (posref
    :reader posref
    :initarg :posref
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (angles
    :reader angles
    :initarg :angles
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (integrator
    :reader integrator
    :initarg :integrator
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass ctrl_debug (<ctrl_debug>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ctrl_debug>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ctrl_debug)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name HbPosCtrl-msg:<ctrl_debug> is deprecated: use HbPosCtrl-msg:ctrl_debug instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ctrl_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HbPosCtrl-msg:header-val is deprecated.  Use HbPosCtrl-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'epos-val :lambda-list '(m))
(cl:defmethod epos-val ((m <ctrl_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HbPosCtrl-msg:epos-val is deprecated.  Use HbPosCtrl-msg:epos instead.")
  (epos m))

(cl:ensure-generic-function 'evel-val :lambda-list '(m))
(cl:defmethod evel-val ((m <ctrl_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HbPosCtrl-msg:evel-val is deprecated.  Use HbPosCtrl-msg:evel instead.")
  (evel m))

(cl:ensure-generic-function 'fdes-val :lambda-list '(m))
(cl:defmethod fdes-val ((m <ctrl_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HbPosCtrl-msg:fdes-val is deprecated.  Use HbPosCtrl-msg:fdes instead.")
  (fdes m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <ctrl_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HbPosCtrl-msg:acc-val is deprecated.  Use HbPosCtrl-msg:acc instead.")
  (acc m))

(cl:ensure-generic-function 'accb-val :lambda-list '(m))
(cl:defmethod accb-val ((m <ctrl_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HbPosCtrl-msg:accb-val is deprecated.  Use HbPosCtrl-msg:accb instead.")
  (accb m))

(cl:ensure-generic-function 'accref-val :lambda-list '(m))
(cl:defmethod accref-val ((m <ctrl_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HbPosCtrl-msg:accref-val is deprecated.  Use HbPosCtrl-msg:accref instead.")
  (accref m))

(cl:ensure-generic-function 'velref-val :lambda-list '(m))
(cl:defmethod velref-val ((m <ctrl_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HbPosCtrl-msg:velref-val is deprecated.  Use HbPosCtrl-msg:velref instead.")
  (velref m))

(cl:ensure-generic-function 'posref-val :lambda-list '(m))
(cl:defmethod posref-val ((m <ctrl_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HbPosCtrl-msg:posref-val is deprecated.  Use HbPosCtrl-msg:posref instead.")
  (posref m))

(cl:ensure-generic-function 'angles-val :lambda-list '(m))
(cl:defmethod angles-val ((m <ctrl_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HbPosCtrl-msg:angles-val is deprecated.  Use HbPosCtrl-msg:angles instead.")
  (angles m))

(cl:ensure-generic-function 'integrator-val :lambda-list '(m))
(cl:defmethod integrator-val ((m <ctrl_debug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader HbPosCtrl-msg:integrator-val is deprecated.  Use HbPosCtrl-msg:integrator instead.")
  (integrator m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ctrl_debug>) ostream)
  "Serializes a message object of type '<ctrl_debug>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'epos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'evel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'fdes) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acc) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accb) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'posref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angles) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'integrator) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ctrl_debug>) istream)
  "Deserializes a message object of type '<ctrl_debug>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'epos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'evel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'fdes) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acc) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accb) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'posref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angles) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'integrator) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ctrl_debug>)))
  "Returns string type for a message object of type '<ctrl_debug>"
  "HbPosCtrl/ctrl_debug")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ctrl_debug)))
  "Returns string type for a message object of type 'ctrl_debug"
  "HbPosCtrl/ctrl_debug")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ctrl_debug>)))
  "Returns md5sum for a message object of type '<ctrl_debug>"
  "f7891061b6e899323360de7ce885079e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ctrl_debug)))
  "Returns md5sum for a message object of type 'ctrl_debug"
  "f7891061b6e899323360de7ce885079e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ctrl_debug>)))
  "Returns full string definition for message of type '<ctrl_debug>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point epos~%geometry_msgs/Point evel~%geometry_msgs/Point fdes~%geometry_msgs/Point acc~%geometry_msgs/Point accb~%geometry_msgs/Point accref~%geometry_msgs/Point velref~%geometry_msgs/Point posref~%geometry_msgs/Point angles~%geometry_msgs/Point integrator~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ctrl_debug)))
  "Returns full string definition for message of type 'ctrl_debug"
  (cl:format cl:nil "Header header~%geometry_msgs/Point epos~%geometry_msgs/Point evel~%geometry_msgs/Point fdes~%geometry_msgs/Point acc~%geometry_msgs/Point accb~%geometry_msgs/Point accref~%geometry_msgs/Point velref~%geometry_msgs/Point posref~%geometry_msgs/Point angles~%geometry_msgs/Point integrator~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ctrl_debug>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'epos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'evel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'fdes))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acc))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accb))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'posref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angles))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'integrator))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ctrl_debug>))
  "Converts a ROS message object to a list"
  (cl:list 'ctrl_debug
    (cl:cons ':header (header msg))
    (cl:cons ':epos (epos msg))
    (cl:cons ':evel (evel msg))
    (cl:cons ':fdes (fdes msg))
    (cl:cons ':acc (acc msg))
    (cl:cons ':accb (accb msg))
    (cl:cons ':accref (accref msg))
    (cl:cons ':velref (velref msg))
    (cl:cons ':posref (posref msg))
    (cl:cons ':angles (angles msg))
    (cl:cons ':integrator (integrator msg))
))
