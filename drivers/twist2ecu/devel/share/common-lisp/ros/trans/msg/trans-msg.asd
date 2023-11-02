
(cl:in-package :asdf)

(defsystem "trans-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ecu" :depends-on ("_package_ecu"))
    (:file "_package_ecu" :depends-on ("_package"))
  ))