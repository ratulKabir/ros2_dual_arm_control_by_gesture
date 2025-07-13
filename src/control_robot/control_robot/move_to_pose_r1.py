#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Lock, Thread
import numpy as np
import time

from pymoveit2 import MoveIt2, MoveIt2State
from control_robot.robots import arm_6dof_1 as robot

from custom_msgs.msg import DualHandState  # Adjust if needed


class DynamicGoalNode(Node):
    def __init__(self):
        super().__init__("dynamic_pose_goal")

        self.callback_group = ReentrantCallbackGroup()
        self.lock = Lock()
        self.latest_goal = None
        self.previous_goal = None
        self.min_goal_distance = 2

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )


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

    def arm_state_callback(self, msg: DualHandState):
        center = msg.right_hand.center

        new_position = np.array([-center.x, center.y, center.z]) # reverese x-axis to match robot's coordinate system

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