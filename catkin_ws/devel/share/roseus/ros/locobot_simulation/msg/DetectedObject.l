;; Auto-generated. Do not edit!


(when (boundp 'locobot_simulation::DetectedObject)
  (if (not (find-package "LOCOBOT_SIMULATION"))
    (make-package "LOCOBOT_SIMULATION"))
  (shadow 'DetectedObject (find-package "LOCOBOT_SIMULATION")))
(unless (find-package "LOCOBOT_SIMULATION::DETECTEDOBJECT")
  (make-package "LOCOBOT_SIMULATION::DETECTEDOBJECT"))

(in-package "ROS")
;;//! \htmlinclude DetectedObject.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass locobot_simulation::DetectedObject
  :super ros::object
  :slots (_name _label _probability _bbox _color _position _num_points ))

(defmethod locobot_simulation::DetectedObject
  (:init
   (&key
    ((:name __name) "")
    ((:label __label) "")
    ((:probability __probability) 0.0)
    ((:bbox __bbox) (make-array 0 :initial-element 0 :element-type :integer))
    ((:color __color) (instance std_msgs::ColorRGBA :init))
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:num_points __num_points) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _label (string __label))
   (setq _probability (float __probability))
   (setq _bbox __bbox)
   (setq _color __color)
   (setq _position __position)
   (setq _num_points (round __num_points))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:probability
   (&optional __probability)
   (if __probability (setq _probability __probability)) _probability)
  (:bbox
   (&optional __bbox)
   (if __bbox (setq _bbox __bbox)) _bbox)
  (:color
   (&rest __color)
   (if (keywordp (car __color))
       (send* _color __color)
     (progn
       (if __color (setq _color (car __color)))
       _color)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:num_points
   (&optional __num_points)
   (if __num_points (setq _num_points __num_points)) _num_points)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _label
    4 (length _label)
    ;; float64 _probability
    8
    ;; int64[] _bbox
    (* 8    (length _bbox)) 4
    ;; std_msgs/ColorRGBA _color
    (send _color :serialization-length)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; int32 _num_points
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; float64 _probability
       (sys::poke _probability (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64[] _bbox
     (write-long (length _bbox) s)
     (dotimes (i (length _bbox))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _bbox i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _bbox i)) (= (length ((elt _bbox i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _bbox i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _bbox i) . bv) 1) -1) s))
             ((and (class (elt _bbox i)) (= (length ((elt _bbox i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _bbox i) . bv) 0) s)
              (write-long (if (>= (elt _bbox i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _bbox i) s)(write-long (if (>= (elt _bbox i) 0) 0 #xffffffff) s)))
       )
     ;; std_msgs/ColorRGBA _color
       (send _color :serialize s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; int32 _num_points
       (write-long _num_points s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _probability
     (setq _probability (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64[] _bbox
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bbox (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _bbox i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _bbox i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; std_msgs/ColorRGBA _color
     (send _color :deserialize buf ptr-) (incf ptr- (send _color :serialization-length))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; int32 _num_points
     (setq _num_points (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get locobot_simulation::DetectedObject :md5sum-) "24b1559ff07fbef77b52a17ccdb9e43a")
(setf (get locobot_simulation::DetectedObject :datatype-) "locobot_simulation/DetectedObject")
(setf (get locobot_simulation::DetectedObject :definition-)
      "string name
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



(provide :locobot_simulation/DetectedObject "24b1559ff07fbef77b52a17ccdb9e43a")


