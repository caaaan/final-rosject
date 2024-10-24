;; Auto-generated. Do not edit!


(when (boundp 'locobot_simulation::DetectedObjects)
  (if (not (find-package "LOCOBOT_SIMULATION"))
    (make-package "LOCOBOT_SIMULATION"))
  (shadow 'DetectedObjects (find-package "LOCOBOT_SIMULATION")))
(unless (find-package "LOCOBOT_SIMULATION::DETECTEDOBJECTS")
  (make-package "LOCOBOT_SIMULATION::DETECTEDOBJECTS"))

(in-package "ROS")
;;//! \htmlinclude DetectedObjects.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass locobot_simulation::DetectedObjects
  :super ros::object
  :slots (_header _detected_objects ))

(defmethod locobot_simulation::DetectedObjects
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detected_objects __detected_objects) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _detected_objects __detected_objects)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detected_objects
   (&rest __detected_objects)
   (if (keywordp (car __detected_objects))
       (send* _detected_objects __detected_objects)
     (progn
       (if __detected_objects (setq _detected_objects (car __detected_objects)))
       _detected_objects)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; locobot_simulation/DetectedObject[] _detected_objects
    (apply #'+ (send-all _detected_objects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; locobot_simulation/DetectedObject[] _detected_objects
     (write-long (length _detected_objects) s)
     (dolist (elem _detected_objects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; locobot_simulation/DetectedObject[] _detected_objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detected_objects (let (r) (dotimes (i n) (push (instance locobot_simulation::DetectedObject :init) r)) r))
     (dolist (elem- _detected_objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get locobot_simulation::DetectedObjects :md5sum-) "c19f34fdf2b6530ceddefc27ebf8226d")
(setf (get locobot_simulation::DetectedObjects :datatype-) "locobot_simulation/DetectedObjects")
(setf (get locobot_simulation::DetectedObjects :definition-)
      "Header header
DetectedObject[] detected_objects
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: locobot_simulation/DetectedObject
string name
string label
float64 probability
int64[] bbox
std_msgs/ColorRGBA color
geometry_msgs/Point position
int32 num_points
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :locobot_simulation/DetectedObjects "c19f34fdf2b6530ceddefc27ebf8226d")


