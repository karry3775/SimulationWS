; Auto-generated. Do not edit!


(cl:in-package dataspeed_can_msgs-msg)


;//! \htmlinclude CanMessageStamped.msg.html

(cl:defclass <CanMessageStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (msg
    :reader msg
    :initarg :msg
    :type dataspeed_can_msgs-msg:CanMessage
    :initform (cl:make-instance 'dataspeed_can_msgs-msg:CanMessage)))
)

(cl:defclass CanMessageStamped (<CanMessageStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CanMessageStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CanMessageStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dataspeed_can_msgs-msg:<CanMessageStamped> is deprecated: use dataspeed_can_msgs-msg:CanMessageStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CanMessageStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dataspeed_can_msgs-msg:header-val is deprecated.  Use dataspeed_can_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <CanMessageStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dataspeed_can_msgs-msg:msg-val is deprecated.  Use dataspeed_can_msgs-msg:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CanMessageStamped>) ostream)
  "Serializes a message object of type '<CanMessageStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CanMessageStamped>) istream)
  "Deserializes a message object of type '<CanMessageStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CanMessageStamped>)))
  "Returns string type for a message object of type '<CanMessageStamped>"
  "dataspeed_can_msgs/CanMessageStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CanMessageStamped)))
  "Returns string type for a message object of type 'CanMessageStamped"
  "dataspeed_can_msgs/CanMessageStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CanMessageStamped>)))
  "Returns md5sum for a message object of type '<CanMessageStamped>"
  "33747cb98e223cafb806d7e94cb4071f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CanMessageStamped)))
  "Returns md5sum for a message object of type 'CanMessageStamped"
  "33747cb98e223cafb806d7e94cb4071f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CanMessageStamped>)))
  "Returns full string definition for message of type '<CanMessageStamped>"
  (cl:format cl:nil "Header header~%~%CanMessage msg~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: dataspeed_can_msgs/CanMessage~%uint8[8] data~%uint32 id~%bool extended~%uint8 dlc~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CanMessageStamped)))
  "Returns full string definition for message of type 'CanMessageStamped"
  (cl:format cl:nil "Header header~%~%CanMessage msg~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: dataspeed_can_msgs/CanMessage~%uint8[8] data~%uint32 id~%bool extended~%uint8 dlc~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CanMessageStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CanMessageStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'CanMessageStamped
    (cl:cons ':header (header msg))
    (cl:cons ':msg (msg msg))
))
