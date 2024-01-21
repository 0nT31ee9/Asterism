; Auto-generated. Do not edit!


(cl:in-package lidar_sim-msg)


;//! \htmlinclude yolo_msg_array.msg.html

(cl:defclass <yolo_msg_array> (roslisp-msg-protocol:ros-message)
  ((yolo_array
    :reader yolo_array
    :initarg :yolo_array
    :type (cl:vector lidar_sim-msg:yolo_msg)
   :initform (cl:make-array 0 :element-type 'lidar_sim-msg:yolo_msg :initial-element (cl:make-instance 'lidar_sim-msg:yolo_msg))))
)

(cl:defclass yolo_msg_array (<yolo_msg_array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <yolo_msg_array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'yolo_msg_array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_sim-msg:<yolo_msg_array> is deprecated: use lidar_sim-msg:yolo_msg_array instead.")))

(cl:ensure-generic-function 'yolo_array-val :lambda-list '(m))
(cl:defmethod yolo_array-val ((m <yolo_msg_array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_sim-msg:yolo_array-val is deprecated.  Use lidar_sim-msg:yolo_array instead.")
  (yolo_array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <yolo_msg_array>) ostream)
  "Serializes a message object of type '<yolo_msg_array>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'yolo_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'yolo_array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <yolo_msg_array>) istream)
  "Deserializes a message object of type '<yolo_msg_array>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'yolo_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'yolo_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'lidar_sim-msg:yolo_msg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<yolo_msg_array>)))
  "Returns string type for a message object of type '<yolo_msg_array>"
  "lidar_sim/yolo_msg_array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'yolo_msg_array)))
  "Returns string type for a message object of type 'yolo_msg_array"
  "lidar_sim/yolo_msg_array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<yolo_msg_array>)))
  "Returns md5sum for a message object of type '<yolo_msg_array>"
  "a085b1524497b4e27e72f595045ab303")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'yolo_msg_array)))
  "Returns md5sum for a message object of type 'yolo_msg_array"
  "a085b1524497b4e27e72f595045ab303")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<yolo_msg_array>)))
  "Returns full string definition for message of type '<yolo_msg_array>"
  (cl:format cl:nil "yolo_msg[] yolo_array~%~%~%================================================================================~%MSG: lidar_sim/yolo_msg~%string label~%int32 x1~%int32 y1~%int32 x2~%int32 y2~%int32 center_x~%int32 center_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'yolo_msg_array)))
  "Returns full string definition for message of type 'yolo_msg_array"
  (cl:format cl:nil "yolo_msg[] yolo_array~%~%~%================================================================================~%MSG: lidar_sim/yolo_msg~%string label~%int32 x1~%int32 y1~%int32 x2~%int32 y2~%int32 center_x~%int32 center_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <yolo_msg_array>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'yolo_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <yolo_msg_array>))
  "Converts a ROS message object to a list"
  (cl:list 'yolo_msg_array
    (cl:cons ':yolo_array (yolo_array msg))
))
