
(cl:in-package :asdf)

(defsystem "motor_control_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ModbusWrite" :depends-on ("_package_ModbusWrite"))
    (:file "_package_ModbusWrite" :depends-on ("_package"))
  ))