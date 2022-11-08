;; Auto-generated. Do not edit!


(when (boundp 'cpr_robot::RobotState)
  (if (not (find-package "CPR_ROBOT"))
    (make-package "CPR_ROBOT"))
  (shadow 'RobotState (find-package "CPR_ROBOT")))
(unless (find-package "CPR_ROBOT::ROBOTSTATE")
  (make-package "CPR_ROBOT::ROBOTSTATE"))

(in-package "ROS")
;;//! \htmlinclude RobotState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cpr_robot::RobotState
  :super ros::object
  :slots (_Header _Override _StatusFlags ))

(defmethod cpr_robot::RobotState
  (:init
   (&key
    ((:Header __Header) (instance std_msgs::Header :init))
    ((:Override __Override) 0.0)
    ((:StatusFlags __StatusFlags) 0)
    )
   (send-super :init)
   (setq _Header __Header)
   (setq _Override (float __Override))
   (setq _StatusFlags (round __StatusFlags))
   self)
  (:Header
   (&rest __Header)
   (if (keywordp (car __Header))
       (send* _Header __Header)
     (progn
       (if __Header (setq _Header (car __Header)))
       _Header)))
  (:Override
   (&optional __Override)
   (if __Override (setq _Override __Override)) _Override)
  (:StatusFlags
   (&optional __StatusFlags)
   (if __StatusFlags (setq _StatusFlags __StatusFlags)) _StatusFlags)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _Header
    (send _Header :serialization-length)
    ;; float64 _Override
    8
    ;; uint32 _StatusFlags
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _Header
       (send _Header :serialize s)
     ;; float64 _Override
       (sys::poke _Override (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint32 _StatusFlags
       (write-long _StatusFlags s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _Header
     (send _Header :deserialize buf ptr-) (incf ptr- (send _Header :serialization-length))
   ;; float64 _Override
     (setq _Override (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint32 _StatusFlags
     (setq _StatusFlags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cpr_robot::RobotState :md5sum-) "9f1e4c5a63d074be8d818e2683b635ac")
(setf (get cpr_robot::RobotState :datatype-) "cpr_robot/RobotState")
(setf (get cpr_robot::RobotState :definition-)
      "std_msgs/Header Header
float64 Override
uint32 StatusFlags
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



(provide :cpr_robot/RobotState "9f1e4c5a63d074be8d818e2683b635ac")


