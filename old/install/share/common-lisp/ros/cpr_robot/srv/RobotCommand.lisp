; Auto-generated. Do not edit!


(cl:in-package cpr_robot-srv)


;//! \htmlinclude RobotCommand-request.msg.html

(cl:defclass <RobotCommand-request> (roslisp-msg-protocol:ros-message)
  ((Sender
    :reader Sender
    :initarg :Sender
    :type cl:string
    :initform "")
   (CommandId
    :reader CommandId
    :initarg :CommandId
    :type cl:integer
    :initform 0)
   (PayloadFloat
    :reader PayloadFloat
    :initarg :PayloadFloat
    :type cl:float
    :initform 0.0)
   (PayloadInt
    :reader PayloadInt
    :initarg :PayloadInt
    :type cl:integer
    :initform 0))
)

(cl:defclass RobotCommand-request (<RobotCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cpr_robot-srv:<RobotCommand-request> is deprecated: use cpr_robot-srv:RobotCommand-request instead.")))

(cl:ensure-generic-function 'Sender-val :lambda-list '(m))
(cl:defmethod Sender-val ((m <RobotCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-srv:Sender-val is deprecated.  Use cpr_robot-srv:Sender instead.")
  (Sender m))

(cl:ensure-generic-function 'CommandId-val :lambda-list '(m))
(cl:defmethod CommandId-val ((m <RobotCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-srv:CommandId-val is deprecated.  Use cpr_robot-srv:CommandId instead.")
  (CommandId m))

(cl:ensure-generic-function 'PayloadFloat-val :lambda-list '(m))
(cl:defmethod PayloadFloat-val ((m <RobotCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-srv:PayloadFloat-val is deprecated.  Use cpr_robot-srv:PayloadFloat instead.")
  (PayloadFloat m))

(cl:ensure-generic-function 'PayloadInt-val :lambda-list '(m))
(cl:defmethod PayloadInt-val ((m <RobotCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-srv:PayloadInt-val is deprecated.  Use cpr_robot-srv:PayloadInt instead.")
  (PayloadInt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotCommand-request>) ostream)
  "Serializes a message object of type '<RobotCommand-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Sender))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Sender))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'CommandId)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'CommandId)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'CommandId)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'CommandId)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'PayloadFloat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'PayloadInt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotCommand-request>) istream)
  "Deserializes a message object of type '<RobotCommand-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Sender) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Sender) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'CommandId)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'CommandId)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'CommandId)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'CommandId)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PayloadFloat) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'PayloadInt) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotCommand-request>)))
  "Returns string type for a service object of type '<RobotCommand-request>"
  "cpr_robot/RobotCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotCommand-request)))
  "Returns string type for a service object of type 'RobotCommand-request"
  "cpr_robot/RobotCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotCommand-request>)))
  "Returns md5sum for a message object of type '<RobotCommand-request>"
  "b4ae46840bac87549d3c8ef9dbcd298d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotCommand-request)))
  "Returns md5sum for a message object of type 'RobotCommand-request"
  "b4ae46840bac87549d3c8ef9dbcd298d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotCommand-request>)))
  "Returns full string definition for message of type '<RobotCommand-request>"
  (cl:format cl:nil "string Sender~%uint32 CommandId~%float64 PayloadFloat~%int64 PayloadInt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotCommand-request)))
  "Returns full string definition for message of type 'RobotCommand-request"
  (cl:format cl:nil "string Sender~%uint32 CommandId~%float64 PayloadFloat~%int64 PayloadInt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotCommand-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Sender))
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotCommand-request
    (cl:cons ':Sender (Sender msg))
    (cl:cons ':CommandId (CommandId msg))
    (cl:cons ':PayloadFloat (PayloadFloat msg))
    (cl:cons ':PayloadInt (PayloadInt msg))
))
;//! \htmlinclude RobotCommand-response.msg.html

(cl:defclass <RobotCommand-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RobotCommand-response (<RobotCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cpr_robot-srv:<RobotCommand-response> is deprecated: use cpr_robot-srv:RobotCommand-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotCommand-response>) ostream)
  "Serializes a message object of type '<RobotCommand-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotCommand-response>) istream)
  "Deserializes a message object of type '<RobotCommand-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotCommand-response>)))
  "Returns string type for a service object of type '<RobotCommand-response>"
  "cpr_robot/RobotCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotCommand-response)))
  "Returns string type for a service object of type 'RobotCommand-response"
  "cpr_robot/RobotCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotCommand-response>)))
  "Returns md5sum for a message object of type '<RobotCommand-response>"
  "b4ae46840bac87549d3c8ef9dbcd298d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotCommand-response)))
  "Returns md5sum for a message object of type 'RobotCommand-response"
  "b4ae46840bac87549d3c8ef9dbcd298d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotCommand-response>)))
  "Returns full string definition for message of type '<RobotCommand-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotCommand-response)))
  "Returns full string definition for message of type 'RobotCommand-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotCommand-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotCommand-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RobotCommand)))
  'RobotCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RobotCommand)))
  'RobotCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotCommand)))
  "Returns string type for a service object of type '<RobotCommand>"
  "cpr_robot/RobotCommand")