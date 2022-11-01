
(cl:in-package :asdf)

(defsystem "cpr_robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ChannelStates" :depends-on ("_package_ChannelStates"))
    (:file "_package_ChannelStates" :depends-on ("_package"))
    (:file "RobotState" :depends-on ("_package_RobotState"))
    (:file "_package_RobotState" :depends-on ("_package"))
  ))