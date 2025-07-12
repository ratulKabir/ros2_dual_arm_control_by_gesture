#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Lock, Thread
import numpy as np
import time

from pymoveit2 import MoveIt2, MoveIt2State
from control_robot.robots import arm_6dof as robot_0
from control_robot.robots import arm_6dof_1 as robot_1
from custom_msgs.msg import DualHandState


class DynamicGoalNode(Node):
    def __init__(self):
        super().__init__("dynamic_pose_goal")

        self.callback_group = ReentrantCallbackGroup()
        self.lock = Lock()
        self.min_goal_distance = 0.05  # meters

        self.quat_xyzw = [0.0, 0.0, 0.0, 1.0]

        # MoveIt2 for left arm
        self.moveit2_0 = MoveIt2(
            node=self,
            joint_names=robot_0.joint_names(),
            base_link_name=robot_0.base_link_name(),
            end_effector_name=robot_0.end_effector_name(),
            group_name=robot_0.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # MoveIt2 for right arm
        self.moveit2_1 = MoveIt2(
            node=self,
            joint_names=robot_1.joint_names(),
            base_link_name=robot_1.base_link_name(),
            end_effector_name=robot_1.end_effector_name(),
            group_name=robot_1.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        self.latest_goal_0 = None
        self.previous_goal_0 = None

        self.latest_goal_1 = None
        self.previous_goal_1 = None

        self.create_subscription(
            DualHandState,
            "/dual_arm/state",
            self.arm_state_callback,
            10,
            callback_group=self.callback_group
        )

        self.get_logger().info("Subscribed to /dual_arm/state")

        self.goal_thread = Thread(target=self.goal_loop, daemon=True)
        self.goal_thread.start()

    def arm_state_callback(self, msg: DualHandState):
        left = msg.left_hand.center
        right = msg.right_hand.center

        new_left = None
        new_right = None

        if not np.isnan(left.x):
            new_left = np.array([-left.x, left.y, left.z])

        if not np.isnan(right.x):
            new_right = np.array([-right.x, right.y, right.z])

        with self.lock:
            if new_left is not None and not np.allclose(new_left, 0.0):
                if self.previous_goal_0 is None or np.linalg.norm(new_left - self.previous_goal_0) >= self.min_goal_distance:
                    self.latest_goal_0 = new_left
                    self.get_logger().info(f"New valid goal for Left arm: {new_left.tolist()}")
                else:
                    self.get_logger().info("Left arm goal is too close to previous. Ignored.")
            else:
                self.get_logger().info("Left arm goal is invalid or zero. Ignored.")

            if new_right is not None and not np.allclose(new_right, 0.0):
                if self.previous_goal_1 is None or np.linalg.norm(new_right - self.previous_goal_1) >= self.min_goal_distance:
                    self.latest_goal_1 = new_right
                    self.get_logger().info(f"New valid goal for Right arm: {new_right.tolist()}")
                else:
                    self.get_logger().info("Right arm goal is too close to previous. Ignored.")
            else:
                self.get_logger().info("Right arm goal is invalid or zero. Ignored.")

    def goal_loop(self):
        while rclpy.ok():
            time.sleep(0.05)

            with self.lock:
                left_goal = self.latest_goal_0
                right_goal = self.latest_goal_1

            if left_goal is not None:
                self.send_goal_if_needed(self.moveit2_0, left_goal, "Left")
                with self.lock:
                    self.previous_goal_0 = left_goal
                    self.latest_goal_0 = None

            if right_goal is not None:
                self.send_goal_if_needed(self.moveit2_1, right_goal, "Right")
                with self.lock:
                    self.previous_goal_1 = right_goal
                    self.latest_goal_1 = None

    def send_goal_if_needed(self, moveit2, position, label):
        if moveit2.query_state() == MoveIt2State.EXECUTING:
            self.get_logger().info(f"{label} arm: Cancelling current goal...")
            moveit2.cancel_execution()

        self.get_logger().info(f"{label} arm: Sending new goal: {position.tolist()}")

        try:
            success = moveit2.move_to_pose(
                position=position.tolist(),
                quat_xyzw=self.quat_xyzw,
                cartesian=False,
            )
            if success:
                self.get_logger().info(f"{label} arm: Successfully moved.")
            else:
                self.get_logger().warn(f"{label} arm: Planning failed.")
        except Exception as e:
            self.get_logger().error(f"{label} arm: Exception - {e}")


def main():
    rclpy.init()
    node = DynamicGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
