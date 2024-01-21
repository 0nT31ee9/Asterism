;; Auto-generated. Do not edit!


(when (boundp 'lidar_sim::shot)
  (if (not (find-package "LIDAR_SIM"))
    (make-package "LIDAR_SIM"))
  (shadow 'shot (find-package "LIDAR_SIM")))
(unless (find-package "LIDAR_SIM::SHOT")
  (make-package "LIDAR_SIM::SHOT"))

(in-package "ROS")
;;//! \htmlinclude shot.msg.html


(defclass lidar_sim::shot
  :super ros::object
  :slots (_shot ))

(defmethod lidar_sim::shot
  (:init
   (&key
    ((:shot __shot) 0)
    )
   (send-super :init)
   (setq _shot (round __shot))
   self)
  (:shot
   (&optional __shot)
   (if __shot (setq _shot __shot)) _shot)
  (:serialization-length
   ()
   (+
    ;; int8 _shot
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _shot
       (write-byte _shot s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _shot
     (setq _shot (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _shot 127) (setq _shot (- _shot 256)))
   ;;
   self)
  )

(setf (get lidar_sim::shot :md5sum-) "08639d6a3e086b1cfe56bdf224f3fbc8")
(setf (get lidar_sim::shot :datatype-) "lidar_sim/shot")
(setf (get lidar_sim::shot :definition-)
      "int8 shot
")



(provide :lidar_sim/shot "08639d6a3e086b1cfe56bdf224f3fbc8")


