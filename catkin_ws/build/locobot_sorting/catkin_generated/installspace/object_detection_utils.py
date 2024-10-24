from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion
from locobot_simulation.msg import BoundingBox, DetectedObject, Model
import math
from tf.transformations import quaternion_from_euler
import rospy

def categorize_object(object_type):
    red_objects = ['red_big_ball', 'red_small_ball', 'red_cylinder', 'red_cube', 'red_big_cylinder']
    blue_objects = ['blue_big_ball', 'blue_small_ball', 'blue_cylinder', 'blue_cube', 'blue_big_cylinder']
    
    if object_type in red_objects:
        return 'red'
    elif object_type in blue_objects:
        return 'blue'
    else:
        return 'unknown'

def is_target_area(object_type):
    mat_objects = ['ground_plane', 'blue_target_area', 'red_target_area']
    return object_type.lower() in mat_objects

def is_graspable(object_type):
    graspable_objects = ['small_ball', 'small_cube', 'small_cylinder']
    return any(obj in object_type.lower() for obj in graspable_objects)

def is_pushable(object_type):
    pushable_objects = ['big_ball', 'big_cylinder']
    return any(obj in object_type.lower() for obj in pushable_objects)

def color_to_string(color: ColorRGBA):
    if color.r > color.b:
        return 'red'
    else:
        return 'blue'

def get_sorting_location(object_color: str):
    if object_color == 'red':
        return Point(0.5, -0.3, 0.0)  # Red mat location
    else:
        return Point(0.5, 0.3, 0.0)   # Blue mat location

def get_object_dimensions(detected_object: DetectedObject):
    if not detected_object.bbox or len(detected_object.bbox) < 4:
        return Vector3(0.05, 0.05, 0.05)  # Default size if bbox is not available
    width = (detected_object.bbox[2] - detected_object.bbox[0]) / 1000.0
    height = (detected_object.bbox[3] - detected_object.bbox[1]) / 1000.0
    depth = min(width, height)  # Assume depth is the smaller of width or height
    return Vector3(width, height, depth)

def create_bounding_box(detected_object: DetectedObject, object_id: int):
    bounding_box = BoundingBox()
    bounding_box.probability = detected_object.probability
    if len(detected_object.bbox) == 4:
        bounding_box.xmin = detected_object.bbox[0]
        bounding_box.ymin = detected_object.bbox[1]
        bounding_box.xmax = detected_object.bbox[2]
        bounding_box.ymax = detected_object.bbox[3]
    else:
        # Fallback to using position if bbox is not available
        bounding_box.xmin = int(detected_object.position.x - 0.05)
        bounding_box.ymin = int(detected_object.position.y - 0.05)
        bounding_box.xmax = int(detected_object.position.x + 0.05)
        bounding_box.ymax = int(detected_object.position.y + 0.05)
    bounding_box.id = min(max(object_id, -32768), 32767)  # Ensure id is within int16 range
    bounding_box.Class = detected_object.name
    return bounding_box
