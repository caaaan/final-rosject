; Auto-generated. Do not edit!


(cl:in-package locobot_simulation-msg)


;//! \htmlinclude Model.msg.html

(cl:defclass <Model> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (size
    :reader size
    :initarg :size
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (min
    :reader min
    :initarg :min
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (max
    :reader max
    :initarg :max
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass Model (<Model>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Model>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Model)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name locobot_simulation-msg:<Model> is deprecated: use locobot_simulation-msg:Model instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Model>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader locobot_simulation-msg:type-val is deprecated.  Use locobot_simulation-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Model>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader locobot_simulation-msg:pose-val is deprecated.  Use locobot_simulation-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <Model>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader locobot_simulation-msg:size-val is deprecated.  Use locobot_simulation-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <Model>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader locobot_simulation-msg:min-val is deprecated.  Use locobot_simulation-msg:min instead.")
  (min m))

(cl:ensure-generic-function 'max-val :lambda-list '(m))
(cl:defmethod max-val ((m <Model>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader locobot_simulation-msg:max-val is deprecated.  Use locobot_simulation-msg:max instead.")
  (max m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Model>) ostream)
  "Serializes a message object of type '<Model>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'size) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Model>) istream)
  "Deserializes a message object of type '<Model>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'size) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Model>)))
  "Returns string type for a message object of type '<Model>"
  "locobot_simulation/Model")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Model)))
  "Returns string type for a message object of type 'Model"
  "locobot_simulation/Model")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Model>)))
  "Returns md5sum for a message object of type '<Model>"
  "c7c9eb7753f1cda18cb3b02c2efc52ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Model)))
  "Returns md5sum for a message object of type 'Model"
  "c7c9eb7753f1cda18cb3b02c2efc52ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Model>)))
  "Returns full string definition for message of type '<Model>"
  (cl:format cl:nil "string type~%geometry_msgs/Pose pose~%geometry_msgs/Vector3 size~%geometry_msgs/Vector3 min~%geometry_msgs/Vector3 max~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Model)))
  "Returns full string definition for message of type 'Model"
  (cl:format cl:nil "string type~%geometry_msgs/Pose pose~%geometry_msgs/Vector3 size~%geometry_msgs/Vector3 min~%geometry_msgs/Vector3 max~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Model>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'type))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'size))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Model>))
  "Converts a ROS message object to a list"
  (cl:list 'Model
    (cl:cons ':type (type msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':size (size msg))
    (cl:cons ':min (min msg))
    (cl:cons ':max (max msg))
))
