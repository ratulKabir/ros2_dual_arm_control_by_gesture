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
        # Add debug logging
        self.get_logger().debug(f"Received hand_data: {hand_data}")
        
        msg = DualHandState()
        
        # Initialize with empty/default hand states
        msg.left_hand = HandState()
        msg.right_hand = HandState()
        
        # Set default values
        msg.left_hand.center = Point(x=0.0, y=0.0, z=0.0)
        msg.left_hand.is_gripping = False
        msg.right_hand.center = Point(x=0.0, y=0.0, z=0.0)
        msg.right_hand.is_gripping = False
        
        hands_detected = 0
        
        # Only update if hand_data is not None and not empty
        if hand_data:
            for side in ['Left', 'Right']:
                if side in hand_data and hand_data[side] is not None:
                    try:
                        # Debug: log the grip value and its type
                        grip_value = hand_data[side].get('grip')
                        self.get_logger().debug(f"{side} grip value: {grip_value}, type: {type(grip_value)}")
                        
                        # Get the appropriate hand message reference
                        hand_msg = msg.left_hand if side == 'Left' else msg.right_hand
                        
                        # Safely extract center coordinates
                        if 'center' in hand_data[side] and hand_data[side]['center'] is not None:
                            center = hand_data[side]['center']
                            if len(center) >= 2:
                                hand_msg.center = Point(
                                    x=float(center[0]),
                                    y=float(center[1]),
                                    z=0.0
                                )
                                hands_detected += 1
                                # Log when hand is detected
                                self.get_logger().debug(f"{side} hand detected at ({center[0]:.2f}, {center[1]:.2f})")
                        
                        # Safely extract grip status and ensure it's a boolean
                        if grip_value is not None:
                            # Convert to boolean explicitly
                            hand_msg.is_gripping = bool(grip_value)
                        else:
                            hand_msg.is_gripping = False
                        
                    except (KeyError, IndexError, ValueError, TypeError) as e:
                        self.get_logger().warn(f"Error processing {side} hand data: {e}")
                        continue
        
        # Log publishing information
        if hands_detected > 0:
            self.get_logger().info(f"Publishing dual hand state: {hands_detected} hands detected")
        else:
            self.get_logger().debug("Publishing dual hand state: no hands detected")
        
        # Always publish the message
        self.pub.publish(msg)
        
        # Optional: Log that message was successfully published
        self.get_logger().debug("DualHandState message published successfully")


def main(args=None):
    parser = argparse.ArgumentParser(description="ROS2 dual hand state publisher")
    parser.add_argument('--no-vis', action='store_true', help='Disable OpenCV visualization')
    
    # Parse only the arguments that belong to this script
    parsed_args = parser.parse_args()
    
    # Initialize rclpy with the original args parameter (or None)
    rclpy.init(args=args)
    
    # Use the parsed arguments for your node
    node = HandPublisher(visualize=not parsed_args.no_vis)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
