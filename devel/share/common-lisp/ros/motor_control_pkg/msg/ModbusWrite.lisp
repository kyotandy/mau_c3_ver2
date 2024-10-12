; Auto-generated. Do not edit!


(cl:in-package motor_control_pkg-msg)


;//! \htmlinclude ModbusWrite.msg.html

(cl:defclass <ModbusWrite> (roslisp-msg-protocol:ros-message)
  ((address
    :reader address
    :initarg :address
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (slave_id
    :reader slave_id
    :initarg :slave_id
    :type cl:integer
    :initform 0))
)

(cl:defclass ModbusWrite (<ModbusWrite>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModbusWrite>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModbusWrite)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_control_pkg-msg:<ModbusWrite> is deprecated: use motor_control_pkg-msg:ModbusWrite instead.")))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <ModbusWrite>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_control_pkg-msg:address-val is deprecated.  Use motor_control_pkg-msg:address instead.")
  (address m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ModbusWrite>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_control_pkg-msg:data-val is deprecated.  Use motor_control_pkg-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'slave_id-val :lambda-list '(m))
(cl:defmethod slave_id-val ((m <ModbusWrite>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_control_pkg-msg:slave_id-val is deprecated.  Use motor_control_pkg-msg:slave_id instead.")
  (slave_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModbusWrite>) ostream)
  "Serializes a message object of type '<ModbusWrite>"
  (cl:let* ((signed (cl:slot-value msg 'address)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
  (cl:let* ((signed (cl:slot-value msg 'slave_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModbusWrite>) istream)
  "Deserializes a message object of type '<ModbusWrite>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'address) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'slave_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModbusWrite>)))
  "Returns string type for a message object of type '<ModbusWrite>"
  "motor_control_pkg/ModbusWrite")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModbusWrite)))
  "Returns string type for a message object of type 'ModbusWrite"
  "motor_control_pkg/ModbusWrite")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModbusWrite>)))
  "Returns md5sum for a message object of type '<ModbusWrite>"
  "d9b9443268afca5c5610972c4da160f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModbusWrite)))
  "Returns md5sum for a message object of type 'ModbusWrite"
  "d9b9443268afca5c5610972c4da160f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModbusWrite>)))
  "Returns full string definition for message of type '<ModbusWrite>"
  (cl:format cl:nil "int32 address~%int32[] data~%int32 slave_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModbusWrite)))
  "Returns full string definition for message of type 'ModbusWrite"
  (cl:format cl:nil "int32 address~%int32[] data~%int32 slave_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModbusWrite>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModbusWrite>))
  "Converts a ROS message object to a list"
  (cl:list 'ModbusWrite
    (cl:cons ':address (address msg))
    (cl:cons ':data (data msg))
    (cl:cons ':slave_id (slave_id msg))
))
