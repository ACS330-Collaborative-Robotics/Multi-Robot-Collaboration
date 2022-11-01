; Auto-generated. Do not edit!


(cl:in-package cpr_robot-srv)


;//! \htmlinclude GetJointInfo-request.msg.html

(cl:defclass <GetJointInfo-request> (roslisp-msg-protocol:ros-message)
  ((Sender
    :reader Sender
    :initarg :Sender
    :type cl:string
    :initform "")
   (JointId
    :reader JointId
    :initarg :JointId
    :type cl:integer
    :initform 0))
)

(cl:defclass GetJointInfo-request (<GetJointInfo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointInfo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointInfo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cpr_robot-srv:<GetJointInfo-request> is deprecated: use cpr_robot-srv:GetJointInfo-request instead.")))

(cl:ensure-generic-function 'Sender-val :lambda-list '(m))
(cl:defmethod Sender-val ((m <GetJointInfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-srv:Sender-val is deprecated.  Use cpr_robot-srv:Sender instead.")
  (Sender m))

(cl:ensure-generic-function 'JointId-val :lambda-list '(m))
(cl:defmethod JointId-val ((m <GetJointInfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-srv:JointId-val is deprecated.  Use cpr_robot-srv:JointId instead.")
  (JointId m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointInfo-request>) ostream)
  "Serializes a message object of type '<GetJointInfo-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Sender))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Sender))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'JointId)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'JointId)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'JointId)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'JointId)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointInfo-request>) istream)
  "Deserializes a message object of type '<GetJointInfo-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Sender) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Sender) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'JointId)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'JointId)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'JointId)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'JointId)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointInfo-request>)))
  "Returns string type for a service object of type '<GetJointInfo-request>"
  "cpr_robot/GetJointInfoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointInfo-request)))
  "Returns string type for a service object of type 'GetJointInfo-request"
  "cpr_robot/GetJointInfoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointInfo-request>)))
  "Returns md5sum for a message object of type '<GetJointInfo-request>"
  "06bfa666ccf7ae403dfc3460222a104e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointInfo-request)))
  "Returns md5sum for a message object of type 'GetJointInfo-request"
  "06bfa666ccf7ae403dfc3460222a104e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointInfo-request>)))
  "Returns full string definition for message of type '<GetJointInfo-request>"
  (cl:format cl:nil "string Sender~%uint32 JointId~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointInfo-request)))
  "Returns full string definition for message of type 'GetJointInfo-request"
  (cl:format cl:nil "string Sender~%uint32 JointId~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointInfo-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Sender))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointInfo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointInfo-request
    (cl:cons ':Sender (Sender msg))
    (cl:cons ':JointId (JointId msg))
))
;//! \htmlinclude GetJointInfo-response.msg.html

(cl:defclass <GetJointInfo-response> (roslisp-msg-protocol:ros-message)
  ((JointName
    :reader JointName
    :initarg :JointName
    :type cl:string
    :initform "")
   (JointType
    :reader JointType
    :initarg :JointType
    :type cl:integer
    :initform 0))
)

(cl:defclass GetJointInfo-response (<GetJointInfo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetJointInfo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetJointInfo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cpr_robot-srv:<GetJointInfo-response> is deprecated: use cpr_robot-srv:GetJointInfo-response instead.")))

(cl:ensure-generic-function 'JointName-val :lambda-list '(m))
(cl:defmethod JointName-val ((m <GetJointInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-srv:JointName-val is deprecated.  Use cpr_robot-srv:JointName instead.")
  (JointName m))

(cl:ensure-generic-function 'JointType-val :lambda-list '(m))
(cl:defmethod JointType-val ((m <GetJointInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-srv:JointType-val is deprecated.  Use cpr_robot-srv:JointType instead.")
  (JointType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetJointInfo-response>) ostream)
  "Serializes a message object of type '<GetJointInfo-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'JointName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'JointName))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'JointType)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'JointType)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'JointType)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'JointType)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetJointInfo-response>) istream)
  "Deserializes a message object of type '<GetJointInfo-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'JointName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'JointName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'JointType)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'JointType)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'JointType)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'JointType)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetJointInfo-response>)))
  "Returns string type for a service object of type '<GetJointInfo-response>"
  "cpr_robot/GetJointInfoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointInfo-response)))
  "Returns string type for a service object of type 'GetJointInfo-response"
  "cpr_robot/GetJointInfoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetJointInfo-response>)))
  "Returns md5sum for a message object of type '<GetJointInfo-response>"
  "06bfa666ccf7ae403dfc3460222a104e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetJointInfo-response)))
  "Returns md5sum for a message object of type 'GetJointInfo-response"
  "06bfa666ccf7ae403dfc3460222a104e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetJointInfo-response>)))
  "Returns full string definition for message of type '<GetJointInfo-response>"
  (cl:format cl:nil "string JointName~%uint32 JointType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetJointInfo-response)))
  "Returns full string definition for message of type 'GetJointInfo-response"
  (cl:format cl:nil "string JointName~%uint32 JointType~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetJointInfo-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'JointName))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetJointInfo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetJointInfo-response
    (cl:cons ':JointName (JointName msg))
    (cl:cons ':JointType (JointType msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetJointInfo)))
  'GetJointInfo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetJointInfo)))
  'GetJointInfo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetJointInfo)))
  "Returns string type for a service object of type '<GetJointInfo>"
  "cpr_robot/GetJointInfo")