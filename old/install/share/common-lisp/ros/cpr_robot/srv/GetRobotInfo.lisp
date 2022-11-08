; Auto-generated. Do not edit!


(cl:in-package cpr_robot-srv)


;//! \htmlinclude GetRobotInfo-request.msg.html

(cl:defclass <GetRobotInfo-request> (roslisp-msg-protocol:ros-message)
  ((Sender
    :reader Sender
    :initarg :Sender
    :type cl:string
    :initform ""))
)

(cl:defclass GetRobotInfo-request (<GetRobotInfo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotInfo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotInfo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cpr_robot-srv:<GetRobotInfo-request> is deprecated: use cpr_robot-srv:GetRobotInfo-request instead.")))

(cl:ensure-generic-function 'Sender-val :lambda-list '(m))
(cl:defmethod Sender-val ((m <GetRobotInfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-srv:Sender-val is deprecated.  Use cpr_robot-srv:Sender instead.")
  (Sender m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotInfo-request>) ostream)
  "Serializes a message object of type '<GetRobotInfo-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Sender))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Sender))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotInfo-request>) istream)
  "Deserializes a message object of type '<GetRobotInfo-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Sender) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Sender) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotInfo-request>)))
  "Returns string type for a service object of type '<GetRobotInfo-request>"
  "cpr_robot/GetRobotInfoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotInfo-request)))
  "Returns string type for a service object of type 'GetRobotInfo-request"
  "cpr_robot/GetRobotInfoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotInfo-request>)))
  "Returns md5sum for a message object of type '<GetRobotInfo-request>"
  "e575ccf5296d82bf7ce2a23f1cc16a78")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotInfo-request)))
  "Returns md5sum for a message object of type 'GetRobotInfo-request"
  "e575ccf5296d82bf7ce2a23f1cc16a78")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotInfo-request>)))
  "Returns full string definition for message of type '<GetRobotInfo-request>"
  (cl:format cl:nil "string Sender~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotInfo-request)))
  "Returns full string definition for message of type 'GetRobotInfo-request"
  (cl:format cl:nil "string Sender~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotInfo-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Sender))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotInfo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotInfo-request
    (cl:cons ':Sender (Sender msg))
))
;//! \htmlinclude GetRobotInfo-response.msg.html

(cl:defclass <GetRobotInfo-response> (roslisp-msg-protocol:ros-message)
  ((Model
    :reader Model
    :initarg :Model
    :type cl:string
    :initform "")
   (CountJoints
    :reader CountJoints
    :initarg :CountJoints
    :type cl:integer
    :initform 0)
   (InputChannels
    :reader InputChannels
    :initarg :InputChannels
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (OutputChannels
    :reader OutputChannels
    :initarg :OutputChannels
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass GetRobotInfo-response (<GetRobotInfo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotInfo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotInfo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cpr_robot-srv:<GetRobotInfo-response> is deprecated: use cpr_robot-srv:GetRobotInfo-response instead.")))

(cl:ensure-generic-function 'Model-val :lambda-list '(m))
(cl:defmethod Model-val ((m <GetRobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-srv:Model-val is deprecated.  Use cpr_robot-srv:Model instead.")
  (Model m))

(cl:ensure-generic-function 'CountJoints-val :lambda-list '(m))
(cl:defmethod CountJoints-val ((m <GetRobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-srv:CountJoints-val is deprecated.  Use cpr_robot-srv:CountJoints instead.")
  (CountJoints m))

(cl:ensure-generic-function 'InputChannels-val :lambda-list '(m))
(cl:defmethod InputChannels-val ((m <GetRobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-srv:InputChannels-val is deprecated.  Use cpr_robot-srv:InputChannels instead.")
  (InputChannels m))

(cl:ensure-generic-function 'OutputChannels-val :lambda-list '(m))
(cl:defmethod OutputChannels-val ((m <GetRobotInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-srv:OutputChannels-val is deprecated.  Use cpr_robot-srv:OutputChannels instead.")
  (OutputChannels m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotInfo-response>) ostream)
  "Serializes a message object of type '<GetRobotInfo-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Model))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Model))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'CountJoints)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'CountJoints)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'CountJoints)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'CountJoints)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'InputChannels))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'InputChannels))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'OutputChannels))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'OutputChannels))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotInfo-response>) istream)
  "Deserializes a message object of type '<GetRobotInfo-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Model) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Model) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'CountJoints)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'CountJoints)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'CountJoints)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'CountJoints)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'InputChannels) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'InputChannels)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'OutputChannels) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'OutputChannels)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotInfo-response>)))
  "Returns string type for a service object of type '<GetRobotInfo-response>"
  "cpr_robot/GetRobotInfoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotInfo-response)))
  "Returns string type for a service object of type 'GetRobotInfo-response"
  "cpr_robot/GetRobotInfoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotInfo-response>)))
  "Returns md5sum for a message object of type '<GetRobotInfo-response>"
  "e575ccf5296d82bf7ce2a23f1cc16a78")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotInfo-response)))
  "Returns md5sum for a message object of type 'GetRobotInfo-response"
  "e575ccf5296d82bf7ce2a23f1cc16a78")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotInfo-response>)))
  "Returns full string definition for message of type '<GetRobotInfo-response>"
  (cl:format cl:nil "string Model~%uint32 CountJoints~%string[] InputChannels~%string[] OutputChannels~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotInfo-response)))
  "Returns full string definition for message of type 'GetRobotInfo-response"
  (cl:format cl:nil "string Model~%uint32 CountJoints~%string[] InputChannels~%string[] OutputChannels~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotInfo-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Model))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'InputChannels) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'OutputChannels) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotInfo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotInfo-response
    (cl:cons ':Model (Model msg))
    (cl:cons ':CountJoints (CountJoints msg))
    (cl:cons ':InputChannels (InputChannels msg))
    (cl:cons ':OutputChannels (OutputChannels msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetRobotInfo)))
  'GetRobotInfo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetRobotInfo)))
  'GetRobotInfo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotInfo)))
  "Returns string type for a service object of type '<GetRobotInfo>"
  "cpr_robot/GetRobotInfo")