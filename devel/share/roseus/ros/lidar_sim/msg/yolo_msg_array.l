;; Auto-generated. Do not edit!


(when (boundp 'lidar_sim::yolo_msg_array)
  (if (not (find-package "LIDAR_SIM"))
    (make-package "LIDAR_SIM"))
  (shadow 'yolo_msg_array (find-package "LIDAR_SIM")))
(unless (find-package "LIDAR_SIM::YOLO_MSG_ARRAY")
  (make-package "LIDAR_SIM::YOLO_MSG_ARRAY"))

(in-package "ROS")
;;//! \htmlinclude yolo_msg_array.msg.html


(defclass lidar_sim::yolo_msg_array
  :super ros::object
  :slots (_yolo_array ))

(defmethod lidar_sim::yolo_msg_array
  (:init
   (&key
    ((:yolo_array __yolo_array) ())
    )
   (send-super :init)
   (setq _yolo_array __yolo_array)
   self)
  (:yolo_array
   (&rest __yolo_array)
   (if (keywordp (car __yolo_array))
       (send* _yolo_array __yolo_array)
     (progn
       (if __yolo_array (setq _yolo_array (car __yolo_array)))
       _yolo_array)))
  (:serialization-length
   ()
   (+
    ;; lidar_sim/yolo_msg[] _yolo_array
    (apply #'+ (send-all _yolo_array :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; lidar_sim/yolo_msg[] _yolo_array
     (write-long (length _yolo_array) s)
     (dolist (elem _yolo_array)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; lidar_sim/yolo_msg[] _yolo_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _yolo_array (let (r) (dotimes (i n) (push (instance lidar_sim::yolo_msg :init) r)) r))
     (dolist (elem- _yolo_array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get lidar_sim::yolo_msg_array :md5sum-) "a085b1524497b4e27e72f595045ab303")
(setf (get lidar_sim::yolo_msg_array :datatype-) "lidar_sim/yolo_msg_array")
(setf (get lidar_sim::yolo_msg_array :definition-)
      "yolo_msg[] yolo_array


================================================================================
MSG: lidar_sim/yolo_msg
string label
int32 x1
int32 y1
int32 x2
int32 y2
int32 center_x
int32 center_y

")



(provide :lidar_sim/yolo_msg_array "a085b1524497b4e27e72f595045ab303")


