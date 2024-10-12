;; Auto-generated. Do not edit!


(when (boundp 'motor_control_pkg::ModbusWrite)
  (if (not (find-package "MOTOR_CONTROL_PKG"))
    (make-package "MOTOR_CONTROL_PKG"))
  (shadow 'ModbusWrite (find-package "MOTOR_CONTROL_PKG")))
(unless (find-package "MOTOR_CONTROL_PKG::MODBUSWRITE")
  (make-package "MOTOR_CONTROL_PKG::MODBUSWRITE"))

(in-package "ROS")
;;//! \htmlinclude ModbusWrite.msg.html


(defclass motor_control_pkg::ModbusWrite
  :super ros::object
  :slots (_address _data _slave_id ))

(defmethod motor_control_pkg::ModbusWrite
  (:init
   (&key
    ((:address __address) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    ((:slave_id __slave_id) 0)
    )
   (send-super :init)
   (setq _address (round __address))
   (setq _data __data)
   (setq _slave_id (round __slave_id))
   self)
  (:address
   (&optional __address)
   (if __address (setq _address __address)) _address)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:slave_id
   (&optional __slave_id)
   (if __slave_id (setq _slave_id __slave_id)) _slave_id)
  (:serialization-length
   ()
   (+
    ;; int32 _address
    4
    ;; int32[] _data
    (* 4    (length _data)) 4
    ;; int32 _slave_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _address
       (write-long _address s)
     ;; int32[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-long (elt _data i) s)
       )
     ;; int32 _slave_id
       (write-long _slave_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _address
     (setq _address (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32 _slave_id
     (setq _slave_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get motor_control_pkg::ModbusWrite :md5sum-) "d9b9443268afca5c5610972c4da160f6")
(setf (get motor_control_pkg::ModbusWrite :datatype-) "motor_control_pkg/ModbusWrite")
(setf (get motor_control_pkg::ModbusWrite :definition-)
      "int32 address
int32[] data
int32 slave_id
")



(provide :motor_control_pkg/ModbusWrite "d9b9443268afca5c5610972c4da160f6")


