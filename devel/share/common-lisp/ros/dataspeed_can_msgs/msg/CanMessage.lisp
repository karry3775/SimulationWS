; Auto-generated. Do not edit!


(cl:in-package dataspeed_can_msgs-msg)


;//! \htmlinclude CanMessage.msg.html

(cl:defclass <CanMessage> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 8 :element-type 'cl:fixnum :initial-element 0))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (extended
    :reader extended
    :initarg :extended
    :type cl:boolean
    :initform cl:nil)
   (dlc
    :reader dlc
    :initarg :dlc
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CanMessage (<CanMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CanMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CanMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dataspeed_can_msgs-msg:<CanMessage> is deprecated: use dataspeed_can_msgs-msg:CanMessage instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dataspeed_can_msgs-msg:data-val is deprecated.  Use dataspeed_can_msgs-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dataspeed_can_msgs-msg:id-val is deprecated.  Use dataspeed_can_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'extended-val :lambda-list '(m))
(cl:defmethod extended-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dataspeed_can_msgs-msg:extended-val is deprecated.  Use dataspeed_can_msgs-msg:extended instead.")
  (extended m))

(cl:ensure-generic-function 'dlc-val :lambda-list '(m))
(cl:defmethod dlc-val ((m <CanMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dataspeed_can_msgs-msg:dlc-val is deprecated.  Use dataspeed_can_msgs-msg:dlc instead.")
  (dlc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CanMessage>) ostream)
  "Serializes a message object of type '<CanMessage>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'extended) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dlc)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CanMessage>) istream)
  "Deserializes a message object of type '<CanMessage>"
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 8)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'extended) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dlc)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CanMessage>)))
  "Returns string type for a message object of type '<CanMessage>"
  "dataspeed_can_msgs/CanMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CanMessage)))
  "Returns string type for a message object of type 'CanMessage"
  "dataspeed_can_msgs/CanMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CanMessage>)))
  "Returns md5sum for a message object of type '<CanMessage>"
  "2866d426ed29ed9fab7f393d3ece69b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CanMessage)))
  "Returns md5sum for a message object of type 'CanMessage"
  "2866d426ed29ed9fab7f393d3ece69b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CanMessage>)))
  "Returns full string definition for message of type '<CanMessage>"
  (cl:format cl:nil "uint8[8] data~%uint32 id~%bool extended~%uint8 dlc~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CanMessage)))
  "Returns full string definition for message of type 'CanMessage"
  (cl:format cl:nil "uint8[8] data~%uint32 id~%bool extended~%uint8 dlc~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CanMessage>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CanMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'CanMessage
    (cl:cons ':data (data msg))
    (cl:cons ':id (id msg))
    (cl:cons ':extended (extended msg))
    (cl:cons ':dlc (dlc msg))
))
