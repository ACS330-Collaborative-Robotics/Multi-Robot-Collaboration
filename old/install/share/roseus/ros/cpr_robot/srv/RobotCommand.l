;; Auto-generated. Do not edit!


(when (boundp 'cpr_robot::RobotCommand)
  (if (not (find-package "CPR_ROBOT"))
    (make-package "CPR_ROBOT"))
  (shadow 'RobotCommand (find-package "CPR_ROBOT")))
(unless (find-package "CPR_ROBOT::ROBOTCOMMAND")
  (make-package "CPR_ROBOT::ROBOTCOMMAND"))
(unless (find-package "CPR_ROBOT::ROBOTCOMMANDREQUEST")
  (make-package "CPR_ROBOT::ROBOTCOMMANDREQUEST"))
(unless (find-package "CPR_ROBOT::ROBOTCOMMANDRESPONSE")
  (make-package "CPR_ROBOT::ROBOTCOMMANDRESPONSE"))

(in-package "ROS")





(defclass cpr_robot::RobotCommandRequest
  :super ros::object
  :slots (_Sender _CommandId _PayloadFloat _PayloadInt ))

(defmethod cpr_robot::RobotCommandRequest
  (:init
   (&key
    ((:Sender __Sender) "")
    ((:CommandId __CommandId) 0)
    ((:PayloadFloat __PayloadFloat) 0.0)
    ((:PayloadInt __PayloadInt) 0)
    )
   (send-super :init)
   (setq _Sender (string __Sender))
   (setq _CommandId (round __CommandId))
   (setq _PayloadFloat (float __PayloadFloat))
   (setq _PayloadInt (round __PayloadInt))
   self)
  (:Sender
   (&optional __Sender)
   (if __Sender (setq _Sender __Sender)) _Sender)
  (:CommandId
   (&optional __CommandId)
   (if __CommandId (setq _CommandId __CommandId)) _CommandId)
  (:PayloadFloat
   (&optional __PayloadFloat)
   (if __PayloadFloat (setq _PayloadFloat __PayloadFloat)) _PayloadFloat)
  (:PayloadInt
   (&optional __PayloadInt)
   (if __PayloadInt (setq _PayloadInt __PayloadInt)) _PayloadInt)
  (:serialization-length
   ()
   (+
    ;; string _Sender
    4 (length _Sender)
    ;; uint32 _CommandId
    4
    ;; float64 _PayloadFloat
    8
    ;; int64 _PayloadInt
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Sender
       (write-long (length _Sender) s) (princ _Sender s)
     ;; uint32 _CommandId
       (write-long _CommandId s)
     ;; float64 _PayloadFloat
       (sys::poke _PayloadFloat (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _PayloadInt
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _PayloadInt (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _PayloadInt) (= (length (_PayloadInt . bv)) 2)) ;; bignum
              (write-long (ash (elt (_PayloadInt . bv) 0) 0) s)
              (write-long (ash (elt (_PayloadInt . bv) 1) -1) s))
             ((and (class _PayloadInt) (= (length (_PayloadInt . bv)) 1)) ;; big1
              (write-long (elt (_PayloadInt . bv) 0) s)
              (write-long (if (>= _PayloadInt 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _PayloadInt s)(write-long (if (>= _PayloadInt 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Sender
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Sender (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _CommandId
     (setq _CommandId (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _PayloadFloat
     (setq _PayloadFloat (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _PayloadInt
#+(or :alpha :irix6 :x86_64)
      (setf _PayloadInt (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _PayloadInt (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass cpr_robot::RobotCommandResponse
  :super ros::object
  :slots ())

(defmethod cpr_robot::RobotCommandResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass cpr_robot::RobotCommand
  :super ros::object
  :slots ())

(setf (get cpr_robot::RobotCommand :md5sum-) "b4ae46840bac87549d3c8ef9dbcd298d")
(setf (get cpr_robot::RobotCommand :datatype-) "cpr_robot/RobotCommand")
(setf (get cpr_robot::RobotCommand :request) cpr_robot::RobotCommandRequest)
(setf (get cpr_robot::RobotCommand :response) cpr_robot::RobotCommandResponse)

(defmethod cpr_robot::RobotCommandRequest
  (:response () (instance cpr_robot::RobotCommandResponse :init)))

(setf (get cpr_robot::RobotCommandRequest :md5sum-) "b4ae46840bac87549d3c8ef9dbcd298d")
(setf (get cpr_robot::RobotCommandRequest :datatype-) "cpr_robot/RobotCommandRequest")
(setf (get cpr_robot::RobotCommandRequest :definition-)
      "string Sender
uint32 CommandId
float64 PayloadFloat
int64 PayloadInt
---

")

(setf (get cpr_robot::RobotCommandResponse :md5sum-) "b4ae46840bac87549d3c8ef9dbcd298d")
(setf (get cpr_robot::RobotCommandResponse :datatype-) "cpr_robot/RobotCommandResponse")
(setf (get cpr_robot::RobotCommandResponse :definition-)
      "string Sender
uint32 CommandId
float64 PayloadFloat
int64 PayloadInt
---

")



(provide :cpr_robot/RobotCommand "b4ae46840bac87549d3c8ef9dbcd298d")


