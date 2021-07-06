
(cl:in-package :asdf)

(defsystem "gps_agent_pkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "CalculateIK" :depends-on ("_package_CalculateIK"))
    (:file "_package_CalculateIK" :depends-on ("_package"))
  ))