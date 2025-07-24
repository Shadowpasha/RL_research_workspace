#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import std_msgs.msg
import std_srvs.srv
import time # For sleep

class ResetOnceNode(Node):
    """
    Initializes the ROS 2 node, performs the reset actions once, and then exits.
    """
    def __init__(self):
        # Initialize the ROS 2 node with a unique name
        super().__init__('reset_odometry_once_node')

        self.get_logger().info("Reset Odometry Once Node has started.")

        # Create a publisher for the /reset_odom_serial topic
        # std_msgs.msg.Empty is used because we just need to signal an event, not send data.
        # The second argument is the quality of service (QoS) profile.
        # Here, we use a default "1" for reliable communication.
        self.reset_publisher = self.create_publisher(std_msgs.msg.Empty, '/reset_odom_serial', 1)
        
        # Give the publisher a moment to establish connection
        time.sleep(0.5) 

        # 1. Publish an empty message to the /reset_odom_serial topic
        self.get_logger().info("Publishing reset message to /reset_odom_serial...")
        msg = std_msgs.msg.Empty()
        self.reset_publisher.publish(msg)
        self.get_logger().info("Reset message published.")

        # 2. Call the /reset_odom service (acting as a client)
        # This assumes another node is providing the /reset_odom service.
        self.reset_odom_service_client = self.create_client(std_srvs.srv.Empty, '/reset_odom')
        
        self.get_logger().info("Waiting for /reset_odom service to be available...")
        # Wait until the service is available
        while not self.reset_odom_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        try:
            # Create an empty request for the service
            req = std_srvs.srv.Empty.Request()
            
            # Call the service asynchronously
            self.future = self.reset_odom_service_client.call_async(req)
            
            # Spin until the future is complete (service response is received)
            rclpy.spin_until_future_complete(self, self.future)
            
            if self.future.result() is not None:
                self.get_logger().info("Successfully called /reset_odom service.")
            else:
                self.get_logger().error(f"Service call to /reset_odom failed: {self.future.exception()}")
        except Exception as e:
            self.get_logger().error(f"Service call to /reset_odom failed: {e}")
        
        self.get_logger().info("Reset Odometry Once Node finished its operations and is shutting down.")
        

def main(args=None):
    rclpy.init(args=args)
    node = ResetOnceNode()
    node.destroy_node() # Clean up the node
    rclpy.shutdown()

if __name__ == '__main__':
    main()
