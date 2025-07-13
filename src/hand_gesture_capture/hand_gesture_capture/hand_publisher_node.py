import sys
import argparse

import rclpy
from rclpy.node import Node
from custom_msgs.msg import DualHandState, HandState
from geometry_msgs.msg import Point
from hand_gesture_capture.hand_tracker import HandTracker


class HandPublisher(Node):
    def __init__(self, visualize=True):
        super().__init__('hand_publisher_node')

        self.pub = self.create_publisher(DualHandState, '/dual_arm/state', 10)

        self.get_logger().info("Starting hand tracker...")
        self.tracker = HandTracker(on_update_callback=self.publish_dual_hand_state,
                                   visualize=visualize)
        self.tracker.run()

    def publish_dual_hand_state(self, hand_data):
        self.get_logger().debug(f"Received hand_data: {hand_data}")
        
        msg = DualHandState()
        msg.left_hand = HandState(center=Point(x=0.0, y=0.0, z=0.0), is_gripping=False)
        msg.right_hand = HandState(center=Point(x=0.0, y=0.0, z=0.0), is_gripping=False)
        
        hands_detected = 0

        for side in ['Left', 'Right']:
            if side in hand_data and hand_data[side] is not None:
                try:
                    grip_value = hand_data[side].get('grip')
                    hand_msg = msg.left_hand if side == 'Left' else msg.right_hand

                    if 'center' in hand_data[side] and hand_data[side]['center'] is not None:
                        center = hand_data[side]['center']
                        if len(center) >= 2:
                            hand_msg.center = Point(
                                x=float(center[0]),
                                y=float(center[1]),
                                z=0.0
                            )
                            hands_detected += 1
                            self.get_logger().info(f"{side} hand position: x={center[0]:.2f}, y={center[1]:.2f}")
                    
                    hand_msg.is_gripping = bool(grip_value) if grip_value is not None else False

                except (KeyError, IndexError, ValueError, TypeError) as e:
                    self.get_logger().warn(f"Error processing {side} hand data: {e}")
        
        if hands_detected > 0:
            self.get_logger().info(f"Publishing dual hand state: {hands_detected} hands detected")
        else:
            self.get_logger().debug("Publishing dual hand state: no hands detected")
        
        self.pub.publish(msg)


def main(args=None):
    # Let ROS filter its own args and leave the rest for argparse
    parser = argparse.ArgumentParser(description="ROS2 dual hand state publisher")
    parser.add_argument('--no-vis', action='store_true', help='Disable OpenCV visualization')

    known_args, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = HandPublisher(visualize=not known_args.no_vis)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
