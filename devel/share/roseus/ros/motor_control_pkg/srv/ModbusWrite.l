;; Auto-generated. Do not edit!


(when (boundp 'motor_control_pkg::ModbusWrite)
  (if (not (find-package "MOTOR_CONTROL_PKG"))
    (make-package "MOTOR_CONTROL_PKG"))
  (shadow 'ModbusWrite (find-package "MOTOR_CONTROL_PKG")))
(unless (find-package "MOTOR_CONTROL_PKG::MODBUSWRITE")
  (make-package "MOTOR_CONTROL_PKG::MODBUSWRITE"))
(unless (find-package "MOTOR_CONTROL_PKG::MODBUSWRITEREQUEST")
  (make-package "MOTOR_CONTROL_PKG::MODBUSWRITEREQUEST"))
(unless (find-package "MOTOR_CONTROL_PKG::MODBUSWRITERESPONSE")
  (make-package "MOTOR_CONTROL_PKG::MODBUSWRITERESPONSE"))

(in-package "ROS")





(defclass motor_control_pkg::ModbusWriteRequest
  :super ros::object
  :slots (_address _data _slave_id ))

(defmethod motor_control_pkg::ModbusWriteRequest
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

(defclass motor_control_pkg::ModbusWriteResponse
  :super ros::object
  :slots (_success _message ))

(defmethod motor_control_pkg::ModbusWriteResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass motor_control_pkg::ModbusWrite
  :super ros::object
  :slots ())

(setf (get motor_control_pkg::ModbusWrite :md5sum-) "1d0506667b2278e5081b9ba14a109960")
(setf (get motor_control_pkg::ModbusWrite :datatype-) "motor_control_pkg/ModbusWrite")
(setf (get motor_control_pkg::ModbusWrite :request) motor_control_pkg::ModbusWriteRequest)
(setf (get motor_control_pkg::ModbusWrite :response) motor_control_pkg::ModbusWriteResponse)

(defmethod motor_control_pkg::ModbusWriteRequest
  (:response () (instance motor_control_pkg::ModbusWriteResponse :init)))

(setf (get motor_control_pkg::ModbusWriteRequest :md5sum-) "1d0506667b2278e5081b9ba14a109960")
(setf (get motor_control_pkg::ModbusWriteRequest :datatype-) "motor_control_pkg/ModbusWriteRequest")
(setf (get motor_control_pkg::ModbusWriteRequest :definition-)
      "int32 address
int32[] data
int32 slave_id
---
bool success
string message

")

(setf (get motor_control_pkg::ModbusWriteResponse :md5sum-) "1d0506667b2278e5081b9ba14a109960")
(setf (get motor_control_pkg::ModbusWriteResponse :datatype-) "motor_control_pkg/ModbusWriteResponse")
(setf (get motor_control_pkg::ModbusWriteResponse :definition-)
      "int32 address
int32[] data
int32 slave_id
---
bool success
string message

")



(provide :motor_control_pkg/ModbusWrite "1d0506667b2278e5081b9ba14a109960")


