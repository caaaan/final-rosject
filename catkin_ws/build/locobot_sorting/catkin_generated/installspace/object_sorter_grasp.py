#!/usr/bin/env python3

import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from locobot_simulation.msg import DetectedObjects, DetectedObject, BoundingBoxes
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Vector3
from tf.transformations import quaternion_from_euler
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from object_detection_utils import (
    is_graspable, get_sorting_location, get_object_dimensions
)
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_commander.exception import MoveItCommanderException
import copy
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

class ObjectSorter:
    def __init__(self):
        rospy.init_node('object_sorter', anonymous=True)
        self.robot_name = rospy.get_param("robot_name", "locobot")
        robot = RobotCommander(f"{self.robot_name}/robot_description", ns=self.robot_name)
        
        try:
            rospy.loginfo("Waiting for move_group action server...")
            self.arm_group = MoveGroupCommander("interbotix_arm", robot_description=f"{self.robot_name}/robot_description", ns=self.robot_name, wait_for_servers=20.0)
            self.gripper_group = MoveGroupCommander(
                "interbotix_gripper",
                robot_description=f"{self.robot_name}/robot_description",
                ns=self.robot_name,
                wait_for_servers=20.0
            )
            rospy.loginfo("Successfully connected to move_group action server.")
        except MoveItCommanderException as e:
            rospy.logerr(f"Failed to connect to move_group action server: {e}")
            self.arm_group = self.gripper_group = None
        except Exception as e:
            rospy.logerr(f"Unexpected error when connecting to move_group action server: {e}")
            self.arm_group = self.gripper_group = None
        
        rospy.Subscriber('/detected_objects', DetectedObjects, self.objects_callback)
        self.gripper_pub = rospy.Publisher(f'/{self.robot_name}/gripper_controller/command', JointTrajectory, queue_size=1)
        
        self.objects_to_sort = []
        self.joint_names = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        self.planning_scene = PlanningSceneInterface(ns=self.robot_name)
        
        # Increase planning time and attempts
        if self.arm_group:
            self.arm_group.set_planning_time(10.0)
            self.arm_group.set_num_planning_attempts(10)
            self.arm_group.set_goal_position_tolerance(0.01)
            self.arm_group.set_goal_orientation_tolerance(0.1)
        
        rospy.loginfo("Object Sorter initialized.")

        self.add_table_to_planning_scene()

    def objects_callback(self, data):
        self.objects_to_sort = data.detected_objects
        rospy.loginfo(f"Received {len(self.objects_to_sort)} objects to sort.")

    def select_next_object(self):
        MIN_CONFIDENCE = 0.5
        graspable_objects = [obj for obj in self.objects_to_sort if is_graspable(obj.name) and obj.probability > MIN_CONFIDENCE]
        if graspable_objects:
            # Sort objects by accessibility (distance and height)
            sorted_objects = sorted(graspable_objects, key=lambda obj: self.accessibility_score(obj))
            selected_object = sorted_objects[0]
            rospy.loginfo(f"Selected graspable object: {selected_object.name} (confidence: {selected_object.probability:.2f}, accessibility score: {self.accessibility_score(selected_object):.2f})")
            return selected_object
        else:
            rospy.loginfo("No graspable small objects found with sufficient confidence.")
            return None

    def accessibility_score(self, obj):
        distance = self.distance_from_base(obj)
        height = obj.position.z
        return distance + (0.5 * height)  # Prioritize closer and lower objects

    def distance_from_base(self, obj):
        # Assuming robot base is at (0, 0, 0)
        return math.sqrt(obj.position.x**2 + obj.position.y**2 + obj.position.z**2)

    def object_size(self, obj):
        # Estimate object size based on bounding box
        width = obj.bbox[2] - obj.bbox[0]
        height = obj.bbox[3] - obj.bbox[1]
        return width * height

    def add_object_to_planning_scene(self, obj: DetectedObject):
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.arm_group.get_planning_frame()
        collision_object.id = f"object_{obj.name}"
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        dimensions = get_object_dimensions(obj)
        box.dimensions = [dimensions.x, dimensions.y, dimensions.z]
        
        object_pose = Pose()
        object_pose.position = obj.position
        object_pose.orientation.w = 1.0
        
        collision_object.primitives = [box]
        collision_object.primitive_poses = [object_pose]
        
        self.planning_scene.add_object(collision_object)

    def move_to_object(self, obj):
        if self.arm_group is None:
            rospy.logwarn("MoveIt! not available. Skipping move to object.")
            return False

        rospy.loginfo(f"Attempting to move to object: {obj.name} at position: {obj.position}")

        pre_grasp_pose = self.get_pre_grasp_pose(obj)
        self.arm_group.set_pose_target(pre_grasp_pose)
        
        rospy.loginfo(f"Planning to pre-grasp pose: {pre_grasp_pose.pose}")
        plan = self.arm_group.plan()
        
        # Check if plan is valid (success is the first element of the tuple)
        if plan[0]:
            rospy.loginfo("Executing plan to pre-grasp pose")
            success = self.arm_group.execute(plan[1], wait=True)
            if success:
                rospy.loginfo("Successfully moved to pre-grasp pose")
                return True
            else:
                rospy.logwarn("Failed to execute plan to pre-grasp pose")
        else:
            rospy.logwarn("Failed to plan path to pre-grasp pose")
        
        return False

    def generate_waypoints(self, obj):
        current_pose = self.arm_group.get_current_pose().pose
        pre_grasp_pose = self.get_pre_grasp_pose(obj).pose

        # Create intermediate waypoints
        waypoints = []
        steps = 5
        for i in range(1, steps + 1):
            waypoint = Pose()
            t = i / steps
            waypoint.position.x = current_pose.position.x + t * (pre_grasp_pose.position.x - current_pose.position.x)
            waypoint.position.y = current_pose.position.y + t * (pre_grasp_pose.position.y - current_pose.position.y)
            waypoint.position.z = current_pose.position.z + t * (pre_grasp_pose.position.z - current_pose.position.z)
            waypoint.orientation = pre_grasp_pose.orientation
            waypoints.append(waypoint)

        return waypoints

    def check_collision(self, obj):
        for other_obj in self.objects_to_sort:
            if other_obj != obj:
                distance = math.sqrt(
                    (obj.position.x - other_obj.position.x)**2 +
                    (obj.position.y - other_obj.position.y)**2 +
                    (obj.position.z - other_obj.position.z)**2
                )
                if distance < 0.15:  # Increased from 0.1 to 0.15
                    return True
        return False

    def get_pre_approach_pose(self, obj):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        distance = self.distance_from_base(obj)
        pose.pose.position.x = obj.position.x * 0.8  # Move 80% of the way to the object
        pose.pose.position.y = obj.position.y * 0.8
        pose.pose.position.z = max(obj.position.z + 0.2, 0.2)  # At least 20cm above the object or table
        q = quaternion_from_euler(0, math.pi/2, math.atan2(obj.position.y, obj.position.x))
        pose.pose.orientation = Quaternion(*q)
        return pose

    def get_pre_grasp_pose(self, obj):
        pose = PoseStamped()
        pose.header.frame_id = self.arm_group.get_planning_frame()
        pose.pose.position = Point(
            obj.position.x - 0.1,  # 10cm back from the object
            obj.position.y,
            max(obj.position.z + 0.1, 0.05)  # At least 5cm above the object or table
        )
        # Calculate orientation to face the object
        direction = Vector3(obj.position.x - pose.pose.position.x,
                            obj.position.y - pose.pose.position.y,
                            obj.position.z - pose.pose.position.z)
        # Use quaternion_from_vector_and_angle to create orientation
        angle = math.atan2(direction.y, direction.x)
        q = quaternion_from_euler(0, math.pi/2, angle)
        pose.pose.orientation = Quaternion(*q)
        return pose

    def get_grasp_pose(self, obj):
        pose = PoseStamped()
        pose.header.frame_id = self.arm_group.get_planning_frame()
        pose.pose.position = Point(
            obj.position.x,
            obj.position.y,
            obj.position.z - 0.1  # Ensure we're not trying to grasp below the table
        )
        # Calculate orientation to face the object
        direction = Vector3(obj.position.x - pose.pose.position.x,
                            obj.position.y - pose.pose.position.y,
                            obj.position.z - pose.pose.position.z)
        # Use quaternion_from_vector_and_angle to create orientation
        angle = math.atan2(direction.y, direction.x)
        q = quaternion_from_euler(0, math.pi/2, angle)
        pose.pose.orientation = Quaternion(*q)
        return pose

    def move_to_pose(self, target_pose):
        self.arm_group.set_pose_target(target_pose)
        self.arm_group.set_planning_time(5.0)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_goal_position_tolerance(0.01)
        self.arm_group.set_goal_orientation_tolerance(0.1)

        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    def grasp_object(self, obj):
        if self.gripper_group is None:
            rospy.logwarn("MoveIt! not available. Skipping grasp object.")
            return False

        rospy.loginfo(f"Attempting to grasp object: {obj.name}")

        # Open the gripper
        if not self.open_gripper():
            rospy.logwarn("Failed to open gripper")
            return False

        # Move to the grasp pose
        grasp_pose = self.get_grasp_pose(obj)
        self.arm_group.set_pose_target(grasp_pose)
        
        rospy.loginfo(f"Planning to grasp pose: {grasp_pose.pose}")
        plan = self.arm_group.plan()
        if not plan[0]:
            rospy.logwarn("Failed to plan path to grasp pose")
            return False

        rospy.loginfo("Executing plan to grasp pose")
        success = False
        try:
            success = self.arm_group.execute(plan[1], wait=True)
        except Exception as e:
            rospy.logerr(f"Exception during grasp execution: {e}")
        
        if not success:
            rospy.logwarn("Failed to execute plan to grasp pose")
            return False

        rospy.loginfo("Successfully moved to grasp pose")

        # Close the gripper
        if not self.close_gripper():
            rospy.logwarn("Failed to close gripper")
            return False

        rospy.loginfo("Successfully grasped object")
        return True

    def move_to_mat(self, target_position: Point):
        target_pose = Pose()
        target_pose.position = Point(target_position.x, target_position.y, target_position.z + 0.1)
        target_pose.orientation = Quaternion(*quaternion_from_euler(0, math.pi/2, 0))

        self.arm_group.set_pose_target(target_pose)
        plan = self.arm_group.plan()
        if plan[0]:
            self.arm_group.execute(plan[1], wait=True)
            return True
        else:
            rospy.logwarn("Failed to plan movement to sorting location")
            return False

    def set_gripper_opening(self, opening):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['left_finger', 'right_finger']
        point = JointTrajectoryPoint()
        point.positions = [opening/2, opening/2]
        point.time_from_start = rospy.Duration(0.5)
        trajectory.points.append(point)
        self.gripper_pub.publish(trajectory)

    def move_to_home_position(self):
        if self.arm_group is None:
            rospy.logwarn("MoveIt! not available. Skipping move to Home position.")
            return False

        self.planning_scene.remove_world_object()
        rospy.sleep(0.5)

        try:
            self.arm_group.set_named_target("Home")
            plan = self.arm_group.plan()
            if plan[0]:
                execute_success = self.arm_group.execute(plan[1], wait=True)
                if execute_success:
                    rospy.loginfo("Successfully moved to Home position")
                    return True
                else:
                    rospy.logwarn("Failed to execute movement to Home position")
            else:
                rospy.logwarn("Failed to plan movement to Home position")
        except Exception as e:
            rospy.logerr(f"Error moving to Home position: {e}")

        return False

    def update_planning_scene(self):
        self.planning_scene.remove_world_object()
        workspace_objects = [obj for obj in self.objects_to_sort if self.is_in_workspace(obj)]
        for obj in workspace_objects:
            self.add_object_to_planning_scene(obj)

    def is_in_workspace(self, obj):
        # Define your workspace limits
        x_min, x_max = 0.5, 1.0
        y_min, y_max = -0.4, 0.4
        z_min, z_max = -0.2, 0.1
        return (x_min <= obj.position.x <= x_max and
                y_min <= obj.position.y <= y_max and
                z_min <= obj.position.z <= z_max)

    def sort_objects(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.update_planning_scene()
            if self.arm_group is None:
                rospy.logwarn("MoveIt! not available. Skipping object sorting.")
                rate.sleep()
                continue

            if not self.move_to_ready_pose():
                rospy.logwarn("Failed to move to Ready pose. Retrying in next iteration.")
                rate.sleep()
                continue

            obj = self.select_next_object()
            if obj:
                rospy.loginfo(f"Attempting to sort object: {obj.name} at position: x={obj.position.x:.2f}, y={obj.position.y:.2f}, z={obj.position.z:.2f}")
                for attempt in range(3):
                    if self.attempt_object_sort(obj):
                        rospy.loginfo(f"Successfully sorted object: {obj.name}")
                        break
                    else:
                        rospy.logwarn(f"Attempt {attempt+1} failed. Retrying...")
                self.move_to_ready_pose()
            else:
                rospy.loginfo("No suitable objects to sort")
                self.move_to_home_pose()
            rate.sleep()

    def run(self):
        rospy.loginfo("Starting object sorter...")
        if self.arm_group is None or self.gripper_group is None:
            rospy.logerr("MoveIt! is not available. Cannot start object sorting.")
            return
        
        self.move_to_home_position()
        self.sort_objects()
        rospy.spin()

    def attempt_object_sort(self, obj):
        if not self.move_to_ready_pose():
            rospy.logwarn("Failed to move to ready pose")
            return False

        if not self.move_to_object(obj):
            rospy.logwarn("Failed to move to object")
            return False

        if not self.grasp_object(obj):
            rospy.logwarn("Failed to grasp object")
            return False

        sorting_location = get_sorting_location(obj.color)
        if not self.move_to_mat(sorting_location):
            rospy.logwarn("Failed to move to sorting location")
            return False

        if not self.open_gripper():  # Open gripper to release object
            rospy.logwarn("Failed to open gripper")
            return False

        rospy.sleep(1.0)

        if not self.move_to_ready_pose():
            rospy.logwarn("Failed to move back to ready pose")
            return False

        return True

    def add_table_to_planning_scene(self):
        table = CollisionObject()
        table.header.frame_id = self.arm_group.get_planning_frame()
        table.id = "table"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.0, 1.0, 0.02]  # Adjust dimensions as needed

        table_pose = Pose()
        table_pose.position.x = 0.5
        table_pose.position.y = 0
        table_pose.position.z = -0.01  # Slightly below the objects
        table_pose.orientation.w = 1.0

        table.primitives = [box]
        table.primitive_poses = [table_pose]

        self.planning_scene.add_object(table)

        # Add sorting areas
        red_area = CollisionObject()
        red_area.header.frame_id = self.arm_group.get_planning_frame()
        red_area.id = "red_sorting_area"
        red_area.primitives = [box]
        red_area_pose = copy.deepcopy(table_pose)
        red_area_pose.position.y = -0.3
        red_area_pose.position.z += 0.02
        red_area.primitive_poses = [red_area_pose]
        self.planning_scene.add_object(red_area)

        blue_area = CollisionObject()
        blue_area.header.frame_id = self.arm_group.get_planning_frame()
        blue_area.id = "blue_sorting_area"
        blue_area.primitives = [box]
        blue_area_pose = copy.deepcopy(table_pose)
        blue_area_pose.position.y = 0.3
        blue_area_pose.position.z += 0.02
        blue_area.primitive_poses = [blue_area_pose]
        self.planning_scene.add_object(blue_area)

    def move_to_named_pose(self, pose_name, group):
        if group == 'arm':
            move_group = self.arm_group
        elif group == 'gripper':
            move_group = self.gripper_group
        else:
            rospy.logwarn(f"Invalid group '{group}' specified. Skipping move to named pose.")
            return False

        if move_group is None:
            rospy.logwarn(f"MoveIt! not available for {group}. Skipping move to named pose.")
            return False

        try:
            move_group.set_named_target(pose_name)
            success = move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()
            return success
        except Exception as e:
            rospy.logerr(f"Error moving to named pose '{pose_name}' for {group}: {e}")
            return False

    
    def move_to_ready_pose(self):
        return self.move_to_named_pose("Ready", group='arm')

    def move_to_home_pose(self):
        return self.move_to_named_pose("Home", group='arm')

    def get_ik_solution(self, target_pose, avoid_collisions=True):
        return self.arm_group.get_current_state().get_joint_group_positions(self.arm_group.get_name())

    def clear_path(self, target_obj):
        for obj in self.objects_to_sort:
            if obj != target_obj and self.is_blocking(obj, target_obj):
                if self.attempt_object_sort(obj):
                    rospy.loginfo(f"Successfully cleared blocking object: {obj.name}")
                    return True
        return False

    def is_blocking(self, obj1, obj2):
        # Check if obj1 is blocking the path to obj2
        distance = math.sqrt(
            (obj1.position.x - obj2.position.x)**2 +
            (obj1.position.y - obj2.position.y)**2
        )
        return distance < 0.2 and obj1.position.z > obj2.position.z

    def open_gripper(self):
        return self.move_to_named_pose("Open", group='gripper')

    def close_gripper(self):
        return self.move_to_named_pose("Closed", group='gripper')

if __name__ == '__main__':
    try:
        sorter = ObjectSorter()
        sorter.run()
    except rospy.ROSInterruptException:
        pass

