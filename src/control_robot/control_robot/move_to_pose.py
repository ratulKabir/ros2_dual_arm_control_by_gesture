#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
import numpy as np
import time

from pymoveit2 import MoveIt2, MoveIt2State
from control_robot.robots import arm_6dof as robot


class DynamicGoalNode(Node):
    def __init__(self):
        super().__init__("dynamic_pose_goal")

        self.callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5

        self.position = np.array([0.25, 0.0, 1.0])
        self.quat_xyzw = [0.0, 0.0, 0.0, 1.0]
        self.rng = np.random.default_rng(seed=42)

        # Start update thread
        self.goal_thread = Thread(target=self.update_loop, daemon=True)
        self.goal_thread.start()

    def update_loop(self):
        while rclpy.ok():
            # Update goal slightly
            self.position += self.rng.uniform(-0.01, 0.01, size=3)

            # Cancel ongoing motion if needed
            if self.moveit2.query_state() == MoveIt2State.EXECUTING:
                self.get_logger().info("Cancelling current goal...")
                self.moveit2.cancel_execution()
                time.sleep(0.1)

            self.get_logger().info(f"New goal: {self.position}")
            self.moveit2.move_to_pose(
                position=self.position.tolist(),
                quat_xyzw=self.quat_xyzw,
                cartesian=False
            )

            time.sleep(0.5)


def main():
    rclpy.init()
    node = DynamicGoalNode()

    # Spin forever
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
