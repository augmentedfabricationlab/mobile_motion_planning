#!/usr/bin/env python3
"""
ROS2 node that subscribes to odometry and processes position data at 10Hz.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from datetime import datetime


class OdomReaderNode(Node):
    """Node that reads odometry data and processes it at 10Hz."""

    def __init__(self):
        super().__init__('odom_reader_node')
        
        # Initialize position variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.data_received = False
        self.exec_index = -1
        
        # Create subscriber to odometry topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/robot/robotnik_base_control/odom',
            self.odom_callback,
            10
        )

        # Subscribe to the UR executed pose index published by ur_pose_streamer_node_live
        self.exec_index_subscriber = self.create_subscription(
            Int32,
            'ur_pose_streamer/exec_index',
            self.exec_index_callback,
            10
        )
        
        # Create timer for 10Hz processing (0.1 seconds = 100ms)
        self.timer = self.create_timer(0.1, self.process_data)
        
        self.get_logger().info('Odom Reader Node started. Subscribing to /robot/robotnik_base_control/odom')
    
    def odom_callback(self, msg):
        """Callback function for odometry messages."""
        # Extract x, y, z position from the odometry message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.data_received = True

    def exec_index_callback(self, msg):
        """Callback for the UR executed pose index."""
        self.exec_index = msg.data
    
    def process_data(self):
        """Process odometry data at 10Hz (called by timer)."""
        if not self.data_received:
            self.get_logger().warn('No odometry data received yet', throttle_duration_sec=5.0)
            return
        
        # Get current timestamp
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        # Print the current position values with timestamp and UR exec index
        print(f"[{timestamp}] exec_idx={self.exec_index} Position: x={self.current_x:.6f}, y={self.current_y:.6f}, z={self.current_z:.6f}")


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    node = OdomReaderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
