#!/usr/bin/env python3

import copy
import actionlib
import rospy
from math import sin, cos
from moveit_python import (MoveGroupInterface, PlanningSceneInterface, PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from locobot_simulation.msg import DetectedObject, DetectedObjects

class GraspingClient(object):
    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("interbotix_arm", "interbotix_gripper", verbose=True)
        self.move_group = MoveGroupInterface("interbotix_arm", "base_link")

        rospy.Subscriber('/detected_objects', DetectedObjects, self.detected_objects_callback)
        self.objects = []

    def detected_objects_callback(self, msg):
        self.objects = msg.detected_objects

    def updateScene(self):
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        for obj in self.objects:
            self.scene.addSolidPrimitive(obj.name, obj.primitives[0], obj.primitive_poses[0], use_service=False)

        self.scene.waitForSync()

    def getGraspableObject(self):
        for obj in self.objects:
            if obj.name in ['red_cube', 'blue_cube', 'red_small_ball', 'blue_small_ball', 'red_cylinder', 'blue_cylinder']:
                return obj, obj.grasps
        return None, None

    def pick(self, block, grasps):
        success, pick_result = self.pickplace.pick_with_retry(block.name, grasps, support_name=block.support_surface, scene=self.scene)
        self.pick_result = pick_result
        return success

    def place(self, block, pose):
        places = []
        l = PlaceLocation()
        l.place_pose = pose
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        m = 16
        pi = 3.141592653589
        for i in range(0, m-1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(l))

        success, place_result = self.pickplace.place_with_retry(block.name, places, scene=self.scene)
        return success

    def tuck(self):
        joints = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def stow(self):
        joints = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
        pose = [1.32, 0.7, 0.0, -2.0, 0.0, -0.57]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def intermediate_stow(self):
        joints = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
        pose = [0.7, -0.3, 0.0, -0.3, 0.0, -0.57]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

if __name__ == "__main__":
    rospy.init_node("demo")
    while not rospy.Time.now():
        pass

    grasping_client = GraspingClient()

    cube_in_grapper = False
    grasping_client.stow()
    pick_place_done = False

    while not rospy.is_shutdown() and not pick_place_done:
        fail_ct = 0
        while not rospy.is_shutdown() and not cube_in_grapper:
            rospy.loginfo("Picking object...")
            grasping_client.updateScene()
            cube, grasps = grasping_client.getGraspableObject()
            if cube is None:
                rospy.logwarn("Perception failed.")
                grasping_client.stow()
                continue

            if grasping_client.pick(cube, grasps):
                cube_in_grapper = True
                break
            rospy.logwarn("Grasping failed.")
            grasping_client.stow()
            if fail_ct > 15:
                fail_ct = 0
                break
            fail_ct += 1

        while not rospy.is_shutdown() and cube_in_grapper:
            rospy.loginfo("Placing object...")
            pose = PoseStamped()
            pose.pose = cube.primitive_poses[0]
            pose.pose.position.y *= -1.0
            pose.pose.position.z += 0.02
            pose.header.frame_id = cube.header.frame_id
            if grasping_client.place(cube, pose):
                cube_in_grapper = False
                break
            rospy.logwarn("Placing failed.")
            grasping_client.intermediate_stow()
            grasping_client.stow()
            if fail_ct > 15:
                fail_ct = 0
                break
            fail_ct += 1

        grasping_client.intermediate_stow()
        grasping_client.stow()
        pick_place_done = True
        rospy.loginfo("Finished")
