;; Auto-generated. Do not edit!


(when (boundp 'cpr_robot::GetRobotInfo)
  (if (not (find-package "CPR_ROBOT"))
    (make-package "CPR_ROBOT"))
  (shadow 'GetRobotInfo (find-package "CPR_ROBOT")))
(unless (find-package "CPR_ROBOT::GETROBOTINFO")
  (make-package "CPR_ROBOT::GETROBOTINFO"))
(unless (find-package "CPR_ROBOT::GETROBOTINFOREQUEST")
  (make-package "CPR_ROBOT::GETROBOTINFOREQUEST"))
(unless (find-package "CPR_ROBOT::GETROBOTINFORESPONSE")
  (make-package "CPR_ROBOT::GETROBOTINFORESPONSE"))

(in-package "ROS")





(defclass cpr_robot::GetRobotInfoRequest
  :super ros::object
  :slots (_Sender ))

(defmethod cpr_robot::GetRobotInfoRequest
  (:init
   (&key
    ((:Sender __Sender) "")
    )
   (send-super :init)
   (setq _Sender (string __Sender))
   self)
  (:Sender
   (&optional __Sender)
   (if __Sender (setq _Sender __Sender)) _Sender)
  (:serialization-length
   ()
   (+
    ;; string _Sender
    4 (length _Sender)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Sender
       (write-long (length _Sender) s) (princ _Sender s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Sender
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Sender (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass cpr_robot::GetRobotInfoResponse
  :super ros::object
  :slots (_Model _CountJoints _InputChannels _OutputChannels ))

(defmethod cpr_robot::GetRobotInfoResponse
  (:init
   (&key
    ((:Model __Model) "")
    ((:CountJoints __CountJoints) 0)
    ((:InputChannels __InputChannels) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:OutputChannels __OutputChannels) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _Model (string __Model))
   (setq _CountJoints (round __CountJoints))
   (setq _InputChannels __InputChannels)
   (setq _OutputChannels __OutputChannels)
   self)
  (:Model
   (&optional __Model)
   (if __Model (setq _Model __Model)) _Model)
  (:CountJoints
   (&optional __CountJoints)
   (if __CountJoints (setq _CountJoints __CountJoints)) _CountJoints)
  (:InputChannels
   (&optional __InputChannels)
   (if __InputChannels (setq _InputChannels __InputChannels)) _InputChannels)
  (:OutputChannels
   (&optional __OutputChannels)
   (if __OutputChannels (setq _OutputChannels __OutputChannels)) _OutputChannels)
  (:serialization-length
   ()
   (+
    ;; string _Model
    4 (length _Model)
    ;; uint32 _CountJoints
    4
    ;; string[] _InputChannels
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _InputChannels)) 4
    ;; string[] _OutputChannels
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _OutputChannels)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Model
       (write-long (length _Model) s) (princ _Model s)
     ;; uint32 _CountJoints
       (write-long _CountJoints s)
     ;; string[] _InputChannels
     (write-long (length _InputChannels) s)
     (dolist (elem _InputChannels)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _OutputChannels
     (write-long (length _OutputChannels) s)
     (dolist (elem _OutputChannels)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Model
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Model (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _CountJoints
     (setq _CountJoints (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string[] _InputChannels
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _InputChannels (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _InputChannels i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _OutputChannels
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _OutputChannels (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _OutputChannels i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass cpr_robot::GetRobotInfo
  :super ros::object
  :slots ())

(setf (get cpr_robot::GetRobotInfo :md5sum-) "e575ccf5296d82bf7ce2a23f1cc16a78")
(setf (get cpr_robot::GetRobotInfo :datatype-) "cpr_robot/GetRobotInfo")
(setf (get cpr_robot::GetRobotInfo :request) cpr_robot::GetRobotInfoRequest)
(setf (get cpr_robot::GetRobotInfo :response) cpr_robot::GetRobotInfoResponse)

(defmethod cpr_robot::GetRobotInfoRequest
  (:response () (instance cpr_robot::GetRobotInfoResponse :init)))

(setf (get cpr_robot::GetRobotInfoRequest :md5sum-) "e575ccf5296d82bf7ce2a23f1cc16a78")
(setf (get cpr_robot::GetRobotInfoRequest :datatype-) "cpr_robot/GetRobotInfoRequest")
(setf (get cpr_robot::GetRobotInfoRequest :definition-)
      "string Sender
---
string Model
uint32 CountJoints
string[] InputChannels
string[] OutputChannels
")

(setf (get cpr_robot::GetRobotInfoResponse :md5sum-) "e575ccf5296d82bf7ce2a23f1cc16a78")
(setf (get cpr_robot::GetRobotInfoResponse :datatype-) "cpr_robot/GetRobotInfoResponse")
(setf (get cpr_robot::GetRobotInfoResponse :definition-)
      "string Sender
---
string Model
uint32 CountJoints
string[] InputChannels
string[] OutputChannels
")



(provide :cpr_robot/GetRobotInfo "e575ccf5296d82bf7ce2a23f1cc16a78")


