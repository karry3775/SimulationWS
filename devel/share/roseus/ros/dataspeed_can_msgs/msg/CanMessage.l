;; Auto-generated. Do not edit!


(when (boundp 'dataspeed_can_msgs::CanMessage)
  (if (not (find-package "DATASPEED_CAN_MSGS"))
    (make-package "DATASPEED_CAN_MSGS"))
  (shadow 'CanMessage (find-package "DATASPEED_CAN_MSGS")))
(unless (find-package "DATASPEED_CAN_MSGS::CANMESSAGE")
  (make-package "DATASPEED_CAN_MSGS::CANMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude CanMessage.msg.html


(defclass dataspeed_can_msgs::CanMessage
  :super ros::object
  :slots (_data _id _extended _dlc ))

(defmethod dataspeed_can_msgs::CanMessage
  (:init
   (&key
    ((:data __data) (make-array 8 :initial-element 0 :element-type :char))
    ((:id __id) 0)
    ((:extended __extended) nil)
    ((:dlc __dlc) 0)
    )
   (send-super :init)
   (setq _data __data)
   (setq _id (round __id))
   (setq _extended __extended)
   (setq _dlc (round __dlc))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:extended
   (&optional __extended)
   (if __extended (setq _extended __extended)) _extended)
  (:dlc
   (&optional __dlc)
   (if __dlc (setq _dlc __dlc)) _dlc)
  (:serialization-length
   ()
   (+
    ;; uint8[8] _data
    (* 1    8)
    ;; uint32 _id
    4
    ;; bool _extended
    1
    ;; uint8 _dlc
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[8] _data
     (princ _data s)
     ;; uint32 _id
       (write-long _id s)
     ;; bool _extended
       (if _extended (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _dlc
       (write-byte _dlc s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[8] _data
   (setq _data (make-array 8 :element-type :char))
   (replace _data buf :start2 ptr-) (incf ptr- 8)
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _extended
     (setq _extended (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _dlc
     (setq _dlc (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get dataspeed_can_msgs::CanMessage :md5sum-) "2866d426ed29ed9fab7f393d3ece69b0")
(setf (get dataspeed_can_msgs::CanMessage :datatype-) "dataspeed_can_msgs/CanMessage")
(setf (get dataspeed_can_msgs::CanMessage :definition-)
      "uint8[8] data
uint32 id
bool extended
uint8 dlc

")



(provide :dataspeed_can_msgs/CanMessage "2866d426ed29ed9fab7f393d3ece69b0")


