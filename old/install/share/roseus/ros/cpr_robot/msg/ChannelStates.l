;; Auto-generated. Do not edit!


(when (boundp 'cpr_robot::ChannelStates)
  (if (not (find-package "CPR_ROBOT"))
    (make-package "CPR_ROBOT"))
  (shadow 'ChannelStates (find-package "CPR_ROBOT")))
(unless (find-package "CPR_ROBOT::CHANNELSTATES")
  (make-package "CPR_ROBOT::CHANNELSTATES"))

(in-package "ROS")
;;//! \htmlinclude ChannelStates.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cpr_robot::ChannelStates
  :super ros::object
  :slots (_Header _state ))

(defmethod cpr_robot::ChannelStates
  (:init
   (&key
    ((:Header __Header) (instance std_msgs::Header :init))
    ((:state __state) (let (r) (dotimes (i 0) (push nil r)) r))
    )
   (send-super :init)
   (setq _Header __Header)
   (setq _state __state)
   self)
  (:Header
   (&rest __Header)
   (if (keywordp (car __Header))
       (send* _Header __Header)
     (progn
       (if __Header (setq _Header (car __Header)))
       _Header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _Header
    (send _Header :serialization-length)
    ;; bool[] _state
    (* 1    (length _state)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _Header
       (send _Header :serialize s)
     ;; bool[] _state
     (write-long (length _state) s)
     (dotimes (i (length _state))
       (if (elt _state i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _Header
     (send _Header :deserialize buf ptr-) (incf ptr- (send _Header :serialization-length))
   ;; bool[] _state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _state (make-list n))
     (dotimes (i n)
     (setf (elt _state i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;;
   self)
  )

(setf (get cpr_robot::ChannelStates :md5sum-) "3787f287236939db0a71aa80023fe8b9")
(setf (get cpr_robot::ChannelStates :datatype-) "cpr_robot/ChannelStates")
(setf (get cpr_robot::ChannelStates :definition-)
      "std_msgs/Header Header
bool[] state

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :cpr_robot/ChannelStates "3787f287236939db0a71aa80023fe8b9")


