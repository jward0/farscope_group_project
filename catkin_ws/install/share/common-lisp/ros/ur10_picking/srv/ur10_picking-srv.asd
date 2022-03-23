
(cl:in-package :asdf)

(defsystem "ur10_picking-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "vacuum_calibration" :depends-on ("_package_vacuum_calibration"))
    (:file "_package_vacuum_calibration" :depends-on ("_package"))
    (:file "vacuum_switch" :depends-on ("_package_vacuum_switch"))
    (:file "_package_vacuum_switch" :depends-on ("_package"))
  ))