;; Auto-generated. Do not edit!


(when (boundp 'dataspeed_can_msgs::CanMessageStamped)
  (if (not (find-package "DATASPEED_CAN_MSGS"))
    (make-package "DATASPEED_CAN_MSGS"))
  (shadow 'CanMessageStamped (find-package "DATASPEED_CAN_MSGS")))
(unless (find-package "DATASPEED_CAN_MSGS::CANMESSAGESTAMPED")
  (make-package "DATASPEED_CAN_MSGS::CANMESSAGESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude CanMessageStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dataspeed_can_msgs::CanMessageStamped
  :super ros::object
  :slots (_header _msg ))

(defmethod dataspeed_can_msgs::CanMessageStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:msg __msg) (instance dataspeed_can_msgs::CanMessage :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _msg __msg)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:msg
   (&rest __msg)
   (if (keywordp (car __msg))
       (send* _msg __msg)
     (progn
       (if __msg (setq _msg (car __msg)))
       _msg)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; dataspeed_can_msgs/CanMessage _msg
    (send _msg :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; dataspeed_can_msgs/CanMessage _msg
       (send _msg :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; dataspeed_can_msgs/CanMessage _msg
     (send _msg :deserialize buf ptr-) (incf ptr- (send _msg :serialization-length))
   ;;
   self)
  )

(setf (get dataspeed_can_msgs::CanMessageStamped :md5sum-) "33747cb98e223cafb806d7e94cb4071f")
(setf (get dataspeed_can_msgs::CanMessageStamped :datatype-) "dataspeed_can_msgs/CanMessageStamped")
(setf (get dataspeed_can_msgs::CanMessageStamped :definition-)
      "Header header

CanMessage msg

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: dataspeed_can_msgs/CanMessage
uint8[8] data
uint32 id
bool extended
uint8 dlc

")



(provide :dataspeed_can_msgs/CanMessageStamped "33747cb98e223cafb806d7e94cb4071f")


