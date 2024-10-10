; Auto-generated. Do not edit!


(cl:in-package motor_control_pkg-srv)


;//! \htmlinclude ModbusWrite-request.msg.html

(cl:defclass <ModbusWrite-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass ModbusWrite-request (<ModbusWrite-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModbusWrite-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModbusWrite-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_control_pkg-srv:<ModbusWrite-request> is deprecated: use motor_control_pkg-srv:ModbusWrite-request instead.")))

(cl:ensure-generic-function 'address-val :lambda-list '(m))
(cl:defmethod address-val ((m <ModbusWrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_control_pkg-srv:address-val is deprecated.  Use motor_control_pkg-srv:address instead.")
  (address m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ModbusWrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_control_pkg-srv:data-val is deprecated.  Use motor_control_pkg-srv:data instead.")
  (data m))

(cl:ensure-generic-function 'slave_id-val :lambda-list '(m))
(cl:defmethod slave_id-val ((m <ModbusWrite-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_control_pkg-srv:slave_id-val is deprecated.  Use motor_control_pkg-srv:slave_id instead.")
  (slave_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModbusWrite-request>) ostream)
  "Serializes a message object of type '<ModbusWrite-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModbusWrite-request>) istream)
  "Deserializes a message object of type '<ModbusWrite-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModbusWrite-request>)))
  "Returns string type for a service object of type '<ModbusWrite-request>"
  "motor_control_pkg/ModbusWriteRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModbusWrite-request)))
  "Returns string type for a service object of type 'ModbusWrite-request"
  "motor_control_pkg/ModbusWriteRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModbusWrite-request>)))
  "Returns md5sum for a message object of type '<ModbusWrite-request>"
  "1d0506667b2278e5081b9ba14a109960")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModbusWrite-request)))
  "Returns md5sum for a message object of type 'ModbusWrite-request"
  "1d0506667b2278e5081b9ba14a109960")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModbusWrite-request>)))
  "Returns full string definition for message of type '<ModbusWrite-request>"
  (cl:format cl:nil "int32 address~%int32[] data~%int32 slave_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModbusWrite-request)))
  "Returns full string definition for message of type 'ModbusWrite-request"
  (cl:format cl:nil "int32 address~%int32[] data~%int32 slave_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModbusWrite-request>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModbusWrite-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ModbusWrite-request
    (cl:cons ':address (address msg))
    (cl:cons ':data (data msg))
    (cl:cons ':slave_id (slave_id msg))
))
;//! \htmlinclude ModbusWrite-response.msg.html

(cl:defclass <ModbusWrite-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass ModbusWrite-response (<ModbusWrite-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModbusWrite-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModbusWrite-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_control_pkg-srv:<ModbusWrite-response> is deprecated: use motor_control_pkg-srv:ModbusWrite-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ModbusWrite-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_control_pkg-srv:success-val is deprecated.  Use motor_control_pkg-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ModbusWrite-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_control_pkg-srv:message-val is deprecated.  Use motor_control_pkg-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModbusWrite-response>) ostream)
  "Serializes a message object of type '<ModbusWrite-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModbusWrite-response>) istream)
  "Deserializes a message object of type '<ModbusWrite-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModbusWrite-response>)))
  "Returns string type for a service object of type '<ModbusWrite-response>"
  "motor_control_pkg/ModbusWriteResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModbusWrite-response)))
  "Returns string type for a service object of type 'ModbusWrite-response"
  "motor_control_pkg/ModbusWriteResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModbusWrite-response>)))
  "Returns md5sum for a message object of type '<ModbusWrite-response>"
  "1d0506667b2278e5081b9ba14a109960")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModbusWrite-response)))
  "Returns md5sum for a message object of type 'ModbusWrite-response"
  "1d0506667b2278e5081b9ba14a109960")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModbusWrite-response>)))
  "Returns full string definition for message of type '<ModbusWrite-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModbusWrite-response)))
  "Returns full string definition for message of type 'ModbusWrite-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModbusWrite-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModbusWrite-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ModbusWrite-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ModbusWrite)))
  'ModbusWrite-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ModbusWrite)))
  'ModbusWrite-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModbusWrite)))
  "Returns string type for a service object of type '<ModbusWrite>"
  "motor_control_pkg/ModbusWrite")