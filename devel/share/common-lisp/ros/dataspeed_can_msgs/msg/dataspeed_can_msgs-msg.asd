
(cl:in-package :asdf)

(defsystem "dataspeed_can_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CanMessage" :depends-on ("_package_CanMessage"))
    (:file "_package_CanMessage" :depends-on ("_package"))
    (:file "CanMessageStamped" :depends-on ("_package_CanMessageStamped"))
    (:file "_package_CanMessageStamped" :depends-on ("_package"))
  ))