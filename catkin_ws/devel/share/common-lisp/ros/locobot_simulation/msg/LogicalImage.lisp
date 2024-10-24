; Auto-generated. Do not edit!


(cl:in-package locobot_simulation-msg)


;//! \htmlinclude LogicalImage.msg.html

(cl:defclass <LogicalImage> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (models
    :reader models
    :initarg :models
    :type (cl:vector locobot_simulation-msg:Model)
   :initform (cl:make-array 0 :element-type 'locobot_simulation-msg:Model :initial-element (cl:make-instance 'locobot_simulation-msg:Model)))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass LogicalImage (<LogicalImage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LogicalImage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LogicalImage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name locobot_simulation-msg:<LogicalImage> is deprecated: use locobot_simulation-msg:LogicalImage instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LogicalImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader locobot_simulation-msg:header-val is deprecated.  Use locobot_simulation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'models-val :lambda-list '(m))
(cl:defmethod models-val ((m <LogicalImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader locobot_simulation-msg:models-val is deprecated.  Use locobot_simulation-msg:models instead.")
  (models m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <LogicalImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader locobot_simulation-msg:pose-val is deprecated.  Use locobot_simulation-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LogicalImage>) ostream)
  "Serializes a message object of type '<LogicalImage>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'models))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'models))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LogicalImage>) istream)
  "Deserializes a message object of type '<LogicalImage>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'models) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'models)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'locobot_simulation-msg:Model))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LogicalImage>)))
  "Returns string type for a message object of type '<LogicalImage>"
  "locobot_simulation/LogicalImage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LogicalImage)))
  "Returns string type for a message object of type 'LogicalImage"
  "locobot_simulation/LogicalImage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LogicalImage>)))
  "Returns md5sum for a message object of type '<LogicalImage>"
  "ce67e5d7268d09846f46b13669044ed7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LogicalImage)))
  "Returns md5sum for a message object of type 'LogicalImage"
  "ce67e5d7268d09846f46b13669044ed7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LogicalImage>)))
  "Returns full string definition for message of type '<LogicalImage>"
  (cl:format cl:nil "std_msgs/Header header~%Model[] models~%geometry_msgs/Pose pose~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: locobot_simulation/Model~%string type~%geometry_msgs/Pose pose~%geometry_msgs/Vector3 size~%geometry_msgs/Vector3 min~%geometry_msgs/Vector3 max~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LogicalImage)))
  "Returns full string definition for message of type 'LogicalImage"
  (cl:format cl:nil "std_msgs/Header header~%Model[] models~%geometry_msgs/Pose pose~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: locobot_simulation/Model~%string type~%geometry_msgs/Pose pose~%geometry_msgs/Vector3 size~%geometry_msgs/Vector3 min~%geometry_msgs/Vector3 max~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LogicalImage>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'models) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LogicalImage>))
  "Converts a ROS message object to a list"
  (cl:list 'LogicalImage
    (cl:cons ':header (header msg))
    (cl:cons ':models (models msg))
    (cl:cons ':pose (pose msg))
))
