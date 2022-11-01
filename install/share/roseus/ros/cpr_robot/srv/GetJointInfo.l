;; Auto-generated. Do not edit!


(when (boundp 'cpr_robot::GetJointInfo)
  (if (not (find-package "CPR_ROBOT"))
    (make-package "CPR_ROBOT"))
  (shadow 'GetJointInfo (find-package "CPR_ROBOT")))
(unless (find-package "CPR_ROBOT::GETJOINTINFO")
  (make-package "CPR_ROBOT::GETJOINTINFO"))
(unless (find-package "CPR_ROBOT::GETJOINTINFOREQUEST")
  (make-package "CPR_ROBOT::GETJOINTINFOREQUEST"))
(unless (find-package "CPR_ROBOT::GETJOINTINFORESPONSE")
  (make-package "CPR_ROBOT::GETJOINTINFORESPONSE"))

(in-package "ROS")





(defclass cpr_robot::GetJointInfoRequest
  :super ros::object
  :slots (_Sender _JointId ))

(defmethod cpr_robot::GetJointInfoRequest
  (:init
   (&key
    ((:Sender __Sender) "")
    ((:JointId __JointId) 0)
    )
   (send-super :init)
   (setq _Sender (string __Sender))
   (setq _JointId (round __JointId))
   self)
  (:Sender
   (&optional __Sender)
   (if __Sender (setq _Sender __Sender)) _Sender)
  (:JointId
   (&optional __JointId)
   (if __JointId (setq _JointId __JointId)) _JointId)
  (:serialization-length
   ()
   (+
    ;; string _Sender
    4 (length _Sender)
    ;; uint32 _JointId
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Sender
       (write-long (length _Sender) s) (princ _Sender s)
     ;; uint32 _JointId
       (write-long _JointId s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Sender
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Sender (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _JointId
     (setq _JointId (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass cpr_robot::GetJointInfoResponse
  :super ros::object
  :slots (_JointName _JointType ))

(defmethod cpr_robot::GetJointInfoResponse
  (:init
   (&key
    ((:JointName __JointName) "")
    ((:JointType __JointType) 0)
    )
   (send-super :init)
   (setq _JointName (string __JointName))
   (setq _JointType (round __JointType))
   self)
  (:JointName
   (&optional __JointName)
   (if __JointName (setq _JointName __JointName)) _JointName)
  (:JointType
   (&optional __JointType)
   (if __JointType (setq _JointType __JointType)) _JointType)
  (:serialization-length
   ()
   (+
    ;; string _JointName
    4 (length _JointName)
    ;; uint32 _JointType
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _JointName
       (write-long (length _JointName) s) (princ _JointName s)
     ;; uint32 _JointType
       (write-long _JointType s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _JointName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _JointName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _JointType
     (setq _JointType (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass cpr_robot::GetJointInfo
  :super ros::object
  :slots ())

(setf (get cpr_robot::GetJointInfo :md5sum-) "06bfa666ccf7ae403dfc3460222a104e")
(setf (get cpr_robot::GetJointInfo :datatype-) "cpr_robot/GetJointInfo")
(setf (get cpr_robot::GetJointInfo :request) cpr_robot::GetJointInfoRequest)
(setf (get cpr_robot::GetJointInfo :response) cpr_robot::GetJointInfoResponse)

(defmethod cpr_robot::GetJointInfoRequest
  (:response () (instance cpr_robot::GetJointInfoResponse :init)))

(setf (get cpr_robot::GetJointInfoRequest :md5sum-) "06bfa666ccf7ae403dfc3460222a104e")
(setf (get cpr_robot::GetJointInfoRequest :datatype-) "cpr_robot/GetJointInfoRequest")
(setf (get cpr_robot::GetJointInfoRequest :definition-)
      "string Sender
uint32 JointId
---
string JointName
uint32 JointType
")

(setf (get cpr_robot::GetJointInfoResponse :md5sum-) "06bfa666ccf7ae403dfc3460222a104e")
(setf (get cpr_robot::GetJointInfoResponse :datatype-) "cpr_robot/GetJointInfoResponse")
(setf (get cpr_robot::GetJointInfoResponse :definition-)
      "string Sender
uint32 JointId
---
string JointName
uint32 JointType
")



(provide :cpr_robot/GetJointInfo "06bfa666ccf7ae403dfc3460222a104e")


