; Auto-generated. Do not edit!


(cl:in-package cpr_robot-msg)


;//! \htmlinclude ChannelStates.msg.html

(cl:defclass <ChannelStates> (roslisp-msg-protocol:ros-message)
  ((Header
    :reader Header
    :initarg :Header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass ChannelStates (<ChannelStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChannelStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChannelStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cpr_robot-msg:<ChannelStates> is deprecated: use cpr_robot-msg:ChannelStates instead.")))

(cl:ensure-generic-function 'Header-val :lambda-list '(m))
(cl:defmethod Header-val ((m <ChannelStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-msg:Header-val is deprecated.  Use cpr_robot-msg:Header instead.")
  (Header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <ChannelStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cpr_robot-msg:state-val is deprecated.  Use cpr_robot-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChannelStates>) ostream)
  "Serializes a message object of type '<ChannelStates>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'state))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChannelStates>) istream)
  "Deserializes a message object of type '<ChannelStates>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'state) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'state)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChannelStates>)))
  "Returns string type for a message object of type '<ChannelStates>"
  "cpr_robot/ChannelStates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChannelStates)))
  "Returns string type for a message object of type 'ChannelStates"
  "cpr_robot/ChannelStates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChannelStates>)))
  "Returns md5sum for a message object of type '<ChannelStates>"
  "3787f287236939db0a71aa80023fe8b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChannelStates)))
  "Returns md5sum for a message object of type 'ChannelStates"
  "3787f287236939db0a71aa80023fe8b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChannelStates>)))
  "Returns full string definition for message of type '<ChannelStates>"
  (cl:format cl:nil "std_msgs/Header Header~%bool[] state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChannelStates)))
  "Returns full string definition for message of type 'ChannelStates"
  (cl:format cl:nil "std_msgs/Header Header~%bool[] state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChannelStates>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChannelStates>))
  "Converts a ROS message object to a list"
  (cl:list 'ChannelStates
    (cl:cons ':Header (Header msg))
    (cl:cons ':state (state msg))
))
