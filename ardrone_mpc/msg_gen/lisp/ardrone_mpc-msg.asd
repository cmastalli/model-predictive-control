
(cl:in-package :asdf)

(defsystem "ardrone_mpc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Reference" :depends-on ("_package_Reference"))
    (:file "_package_Reference" :depends-on ("_package"))
  ))