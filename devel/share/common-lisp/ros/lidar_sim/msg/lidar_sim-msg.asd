
(cl:in-package :asdf)

(defsystem "lidar_sim-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "goal" :depends-on ("_package_goal"))
    (:file "_package_goal" :depends-on ("_package"))
    (:file "shot" :depends-on ("_package_shot"))
    (:file "_package_shot" :depends-on ("_package"))
    (:file "yolo_msg" :depends-on ("_package_yolo_msg"))
    (:file "_package_yolo_msg" :depends-on ("_package"))
    (:file "yolo_msg_array" :depends-on ("_package_yolo_msg_array"))
    (:file "_package_yolo_msg_array" :depends-on ("_package"))
  ))