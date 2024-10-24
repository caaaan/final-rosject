#!/usr/bin/env python3

import os
import sys
import tf2_ros
import tf2_geometry_msgs

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

from object_detection_utils import (
    categorize_object, is_target_area, is_graspable, create_bounding_box
)

import rospy
from locobot_simulation.msg import LogicalImage, DetectedObjects, DetectedObject, BoundingBoxes
from geometry_msgs.msg import Point, PoseStamped
from moveit_commander import MoveGroupCommander, RobotCommander
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA

class CameraObjectDetection:
    def __init__(self):
        rospy.init_node('camera_object_detection', anonymous=True)
        self.robot_name = rospy.get_param("robot_name", "locobot")
        self.detected_objects_publisher = rospy.Publisher('/detected_objects', DetectedObjects, queue_size=10)
        self.detected_objects = []

        robot = RobotCommander(f"{self.robot_name}/robot_description", ns=self.robot_name)
        try:
            self.arm_group = MoveGroupCommander("interbotix_arm", robot_description=f"{self.robot_name}/robot_description", ns=self.robot_name, wait_for_servers=30.0)
            self.gripper_group = MoveGroupCommander("interbotix_gripper", robot_description=f"{self.robot_name}/robot_description", ns=self.robot_name, wait_for_servers=30.0)
            rospy.loginfo("Successfully connected to move_group action server.")
        except RuntimeError as e:
            rospy.logerr(f"Failed to connect to move_group action server: {e}")
            self.arm_group = self.gripper_group = None

        self.default_tilt = 0.6
        self.default_pan = 0.0
        self.mat_locations = {}
        self.camera_ready = False
        rospy.Subscriber("/locobot/joint_states", JointState, self.joint_state_callback)
        rospy.Subscriber("/gazebo/locobot/camera/logical_camera_image", LogicalImage, self.camera_callback)
        self.current_joint_states = None
        rospy.loginfo("Camera Object Detection node initialized.")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def camera_callback(self, data: LogicalImage):
        if not self.camera_ready:
            return

        rospy.loginfo(f"Received camera data with {len(data.models)} models")
        detected_objects = DetectedObjects()
        detected_objects.header = data.header
        self.detected_objects = []

        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = data.header
        bounding_boxes.image_header = data.header

        for i, model in enumerate(data.models):
            if not is_target_area(model.type):
                detected_object = DetectedObject()
                detected_object.name = model.type
                detected_object.label = categorize_object(model.type)
                detected_object.probability = 1.0  # Set a default probability
                detected_object.color = ColorRGBA(1.0, 0.0, 0.0, 1.0) if 'red' in model.type.lower() else ColorRGBA(0.0, 0.0, 1.0, 1.0)
                detected_object.position = model.pose.position
                detected_object.num_points = 1  # Set a default value
                
                # Calculate bounding box from model size and position
                half_size = [model.size.x / 2, model.size.y / 2, model.size.z / 2]
                detected_object.bbox = [
                    int((model.pose.position.x - half_size[0]) * 1000),
                    int((model.pose.position.y - half_size[1]) * 1000),
                    int((model.pose.position.x + half_size[0]) * 1000),
                    int((model.pose.position.y + half_size[1]) * 1000)
                ]

                detected_objects.detected_objects.append(detected_object)
                
                # Create and append BoundingBox
                bounding_box = create_bounding_box(detected_object, i)
                bounding_boxes.bounding_boxes.append(bounding_box)

        if detected_objects.detected_objects:
            self.detected_objects_publisher.publish(detected_objects)
            rospy.loginfo(f"Published {len(detected_objects.detected_objects)} detected objects")
        else:
            rospy.loginfo("No objects detected in this frame")

    def run(self):
        rospy.loginfo("Starting camera object detection...")
        if self.arm_group and self.gripper_group:
            self.move_to_sleep_position()
        self.check_camera_position()
        self.move_camera_to_default_position()
        self.check_camera_position()
        self.camera_ready = True
        rospy.spin()

    def move_to_sleep_position(self):
        rospy.loginfo("Moving to Sleep position for object detection")
        if self.arm_group:
            self.arm_group.set_named_target("Sleep")
            arm_success = self.arm_group.go(wait=True)
            self.arm_group.stop()
            rospy.loginfo("Successfully moved to Sleep position" if arm_success else "Failed to move to Sleep position")
        else:
            rospy.logwarn("MoveIt! not available. Skipping move to Sleep position.")

    def move_camera_to_default_position(self):
        rospy.loginfo(f"Moving camera to default position: pan={self.default_pan}, tilt={self.default_tilt}")
        pan_pub = rospy.Publisher('/locobot/pan_controller/command', Float64, queue_size=1)
        tilt_pub = rospy.Publisher('/locobot/tilt_controller/command', Float64, queue_size=1)
        
        rospy.sleep(1)  # Wait for publishers to register
        
        pan_pub.publish(Float64(self.default_pan))
        tilt_pub.publish(Float64(self.default_tilt))
        
        rospy.sleep(2)  # Wait for movement
        rospy.loginfo("Camera move commands completed")

    def joint_state_callback(self, msg):
        self.current_joint_states = msg

    def check_camera_position(self):
        max_attempts = 5
        attempt = 0
        while attempt < max_attempts:
            try:
                joint_state = rospy.wait_for_message('/locobot/joint_states', JointState, timeout=5.0)
                pan_index = joint_state.name.index('pan')
                tilt_index = joint_state.name.index('tilt')
                current_pan = joint_state.position[pan_index]
                current_tilt = joint_state.position[tilt_index]
                rospy.loginfo(f"Current camera position: pan={current_pan:.4f}, tilt={current_tilt:.4f}")
                return
            except (ValueError, IndexError) as e:
                attempt += 1
                rospy.logwarn(f"Attempt {attempt}/{max_attempts}: Failed to get camera position. Error: {e}")
                rospy.sleep(1.0)  # Wait for 1 second before retrying
    
        rospy.logerr("Failed to get camera position after multiple attempts")

if __name__ == '__main__':
    try:
        node = CameraObjectDetection()
        node.run()
    except rospy.ROSInterruptException:
        pass
