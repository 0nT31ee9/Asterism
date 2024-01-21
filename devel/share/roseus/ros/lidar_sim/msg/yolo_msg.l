;; Auto-generated. Do not edit!


(when (boundp 'lidar_sim::yolo_msg)
  (if (not (find-package "LIDAR_SIM"))
    (make-package "LIDAR_SIM"))
  (shadow 'yolo_msg (find-package "LIDAR_SIM")))
(unless (find-package "LIDAR_SIM::YOLO_MSG")
  (make-package "LIDAR_SIM::YOLO_MSG"))

(in-package "ROS")
;;//! \htmlinclude yolo_msg.msg.html


(defclass lidar_sim::yolo_msg
  :super ros::object
  :slots (_label _x1 _y1 _x2 _y2 _center_x _center_y ))

(defmethod lidar_sim::yolo_msg
  (:init
   (&key
    ((:label __label) "")
    ((:x1 __x1) 0)
    ((:y1 __y1) 0)
    ((:x2 __x2) 0)
    ((:y2 __y2) 0)
    ((:center_x __center_x) 0)
    ((:center_y __center_y) 0)
    )
   (send-super :init)
   (setq _label (string __label))
   (setq _x1 (round __x1))
   (setq _y1 (round __y1))
   (setq _x2 (round __x2))
   (setq _y2 (round __y2))
   (setq _center_x (round __center_x))
   (setq _center_y (round __center_y))
   self)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:y1
   (&optional __y1)
   (if __y1 (setq _y1 __y1)) _y1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:y2
   (&optional __y2)
   (if __y2 (setq _y2 __y2)) _y2)
  (:center_x
   (&optional __center_x)
   (if __center_x (setq _center_x __center_x)) _center_x)
  (:center_y
   (&optional __center_y)
   (if __center_y (setq _center_y __center_y)) _center_y)
  (:serialization-length
   ()
   (+
    ;; string _label
    4 (length _label)
    ;; int32 _x1
    4
    ;; int32 _y1
    4
    ;; int32 _x2
    4
    ;; int32 _y2
    4
    ;; int32 _center_x
    4
    ;; int32 _center_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; int32 _x1
       (write-long _x1 s)
     ;; int32 _y1
       (write-long _y1 s)
     ;; int32 _x2
       (write-long _x2 s)
     ;; int32 _y2
       (write-long _y2 s)
     ;; int32 _center_x
       (write-long _center_x s)
     ;; int32 _center_y
       (write-long _center_y s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _x1
     (setq _x1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y1
     (setq _y1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _x2
     (setq _x2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y2
     (setq _y2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _center_x
     (setq _center_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _center_y
     (setq _center_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get lidar_sim::yolo_msg :md5sum-) "c27fe395966734d5c3bb5d6a88f1f99a")
(setf (get lidar_sim::yolo_msg :datatype-) "lidar_sim/yolo_msg")
(setf (get lidar_sim::yolo_msg :definition-)
      "string label
int32 x1
int32 y1
int32 x2
int32 y2
int32 center_x
int32 center_y

")



(provide :lidar_sim/yolo_msg "c27fe395966734d5c3bb5d6a88f1f99a")


