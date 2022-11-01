
(cl:in-package :asdf)

(defsystem "cpr_robot-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetJointInfo" :depends-on ("_package_GetJointInfo"))
    (:file "_package_GetJointInfo" :depends-on ("_package"))
    (:file "GetRobotInfo" :depends-on ("_package_GetRobotInfo"))
    (:file "_package_GetRobotInfo" :depends-on ("_package"))
    (:file "RobotCommand" :depends-on ("_package_RobotCommand"))
    (:file "_package_RobotCommand" :depends-on ("_package"))
  ))