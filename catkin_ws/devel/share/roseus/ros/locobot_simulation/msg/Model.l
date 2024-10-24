;; Auto-generated. Do not edit!


(when (boundp 'locobot_simulation::Model)
  (if (not (find-package "LOCOBOT_SIMULATION"))
    (make-package "LOCOBOT_SIMULATION"))
  (shadow 'Model (find-package "LOCOBOT_SIMULATION")))
(unless (find-package "LOCOBOT_SIMULATION::MODEL")
  (make-package "LOCOBOT_SIMULATION::MODEL"))

(in-package "ROS")
;;//! \htmlinclude Model.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass locobot_simulation::Model
  :super ros::object
  :slots (_type _pose _size _min _max ))

(defmethod locobot_simulation::Model
  (:init
   (&key
    ((:type __type) "")
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:size __size) (instance geometry_msgs::Vector3 :init))
    ((:min __min) (instance geometry_msgs::Vector3 :init))
    ((:max __max) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _type (string __type))
   (setq _pose __pose)
   (setq _size __size)
   (setq _min __min)
   (setq _max __max)
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:size
   (&rest __size)
   (if (keywordp (car __size))
       (send* _size __size)
     (progn
       (if __size (setq _size (car __size)))
       _size)))
  (:min
   (&rest __min)
   (if (keywordp (car __min))
       (send* _min __min)
     (progn
       (if __min (setq _min (car __min)))
       _min)))
  (:max
   (&rest __max)
   (if (keywordp (car __max))
       (send* _max __max)
     (progn
       (if __max (setq _max (car __max)))
       _max)))
  (:serialization-length
   ()
   (+
    ;; string _type
    4 (length _type)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Vector3 _size
    (send _size :serialization-length)
    ;; geometry_msgs/Vector3 _min
    (send _min :serialization-length)
    ;; geometry_msgs/Vector3 _max
    (send _max :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Vector3 _size
       (send _size :serialize s)
     ;; geometry_msgs/Vector3 _min
       (send _min :serialize s)
     ;; geometry_msgs/Vector3 _max
       (send _max :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Vector3 _size
     (send _size :deserialize buf ptr-) (incf ptr- (send _size :serialization-length))
   ;; geometry_msgs/Vector3 _min
     (send _min :deserialize buf ptr-) (incf ptr- (send _min :serialization-length))
   ;; geometry_msgs/Vector3 _max
     (send _max :deserialize buf ptr-) (incf ptr- (send _max :serialization-length))
   ;;
   self)
  )

(setf (get locobot_simulation::Model :md5sum-) "c7c9eb7753f1cda18cb3b02c2efc52ff")
(setf (get locobot_simulation::Model :datatype-) "locobot_simulation/Model")
(setf (get locobot_simulation::Model :definition-)
      "string type
geometry_msgs/Pose pose
geometry_msgs/Vector3 size
geometry_msgs/Vector3 min
geometry_msgs/Vector3 max

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :locobot_simulation/Model "c7c9eb7753f1cda18cb3b02c2efc52ff")


