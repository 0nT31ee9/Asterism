; Auto-generated. Do not edit!


(cl:in-package lidar_sim-msg)


;//! \htmlinclude shot.msg.html

(cl:defclass <shot> (roslisp-msg-protocol:ros-message)
  ((shot
    :reader shot
    :initarg :shot
    :type cl:fixnum
    :initform 0))
)

(cl:defclass shot (<shot>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <shot>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'shot)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_sim-msg:<shot> is deprecated: use lidar_sim-msg:shot instead.")))

(cl:ensure-generic-function 'shot-val :lambda-list '(m))
(cl:defmethod shot-val ((m <shot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_sim-msg:shot-val is deprecated.  Use lidar_sim-msg:shot instead.")
  (shot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <shot>) ostream)
  "Serializes a message object of type '<shot>"
  (cl:let* ((signed (cl:slot-value msg 'shot)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <shot>) istream)
  "Deserializes a message object of type '<shot>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'shot) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<shot>)))
  "Returns string type for a message object of type '<shot>"
  "lidar_sim/shot")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shot)))
  "Returns string type for a message object of type 'shot"
  "lidar_sim/shot")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<shot>)))
  "Returns md5sum for a message object of type '<shot>"
  "08639d6a3e086b1cfe56bdf224f3fbc8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'shot)))
  "Returns md5sum for a message object of type 'shot"
  "08639d6a3e086b1cfe56bdf224f3fbc8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<shot>)))
  "Returns full string definition for message of type '<shot>"
  (cl:format cl:nil "int8 shot~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'shot)))
  "Returns full string definition for message of type 'shot"
  (cl:format cl:nil "int8 shot~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <shot>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <shot>))
  "Converts a ROS message object to a list"
  (cl:list 'shot
    (cl:cons ':shot (shot msg))
))
