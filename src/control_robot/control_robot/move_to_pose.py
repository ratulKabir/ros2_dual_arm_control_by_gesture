#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Lock, Thread
import numpy as np
import time

from pymoveit2 import MoveIt2, MoveIt2State
from control_robot.robots import left_arm as robot

from custom_msgs.msg import DualHandState  # Adjust if needed


class DynamicGoalNode(Node):
    def __init__(self):
        super().__init__("dynamic_pose_goal")

        self.callback_group = ReentrantCallbackGroup()
        self.lock = Lock()
        self.latest_goal = None
        self.previous_goal = None
        self.min_goal_distance = 0.5  # meters

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # self.moveit2.wait_until_executors_ready()
        # self.get_logger().info("MoveIt2 action server is ready.")


        self.quat_xyzw = [0.0, 0.0, 0.0, 1.0]

        self.create_subscription(
            DualHandState,
            "/dual_arm/state",
            self.arm_state_callback,
            10,
            callback_group=self.callback_group
        )

        self.get_logger().info("Subscribed to /dual_arm/state")

        # Start goal execution loop
        self.goal_thread = Thread(target=self.goal_loop, daemon=True)
        self.goal_thread.start()

    def normalize_point(self, i, j, width=640, height=480, range_val=3.5):
        x_norm = range_val * (2 * j - (width - 1)) / (width - 1)
        y_norm = range_val * ((height - 1) - 2 * i) / (height - 1)
        return x_norm, y_norm

    def arm_state_callback(self, msg: DualHandState):
        left = msg.left_hand.center
        # if left.x == 0.0 and left.y == 0.0 and left.z == 0.0:
        #     return

        x_norm, y_norm = self.normalize_point(left.y, left.x)
        z = left.z

        new_position = np.array([x_norm, y_norm, z])

        with self.lock:
            if (
                self.previous_goal is not None and
                np.linalg.norm(new_position - self.previous_goal) < self.min_goal_distance
            ):
                return
            self.latest_goal = new_position
            self.previous_goal = new_position

        self.get_logger().info(f"Received new valid goal from hand: {new_position.tolist()}")


    def goal_loop(self):
        while rclpy.ok():
            time.sleep(0.05)  # Limit loop frequency

            with self.lock:
                if self.latest_goal is None:
                    continue
                position = self.latest_goal

            if self.moveit2.query_state() == MoveIt2State.EXECUTING:
                self.get_logger().info("Cancelling current goal...")
                self.moveit2.cancel_execution()

            self.get_logger().info(f"Sending new goal: {position.tolist()}")

            try:
                success = self.moveit2.move_to_pose(
                    position=position.tolist(),
                    quat_xyzw=self.quat_xyzw,
                    cartesian=False
                )

                if not success:
                    self.get_logger().warn("Planning failed or trajectory invalid. Skipping this goal.")
                    continue  # Don't clear the goal — loop again and retry if needed

                else:
                    self.get_logger().info("Successfully moved to target pose.")
                    with self.lock:
                        self.previous_goal = position
                        self.latest_goal = None  # Only clear after success

            except Exception as e:
                self.get_logger().error(f"Exception during planning or execution: {e}")



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
