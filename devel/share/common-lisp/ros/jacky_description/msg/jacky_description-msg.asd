
(cl:in-package :asdf)

(defsystem "jacky_description-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DeltaMsgStamped" :depends-on ("_package_DeltaMsgStamped"))
    (:file "_package_DeltaMsgStamped" :depends-on ("_package"))
  ))