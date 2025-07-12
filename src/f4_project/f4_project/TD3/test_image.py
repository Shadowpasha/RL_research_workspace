import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Import the ROS Image message type
from cv_bridge import CvBridge    # Import CvBridge for conversion
import numpy as np
import cv2

class DepthImageSubscriber(Node):
    def __init__(self):
        # Initialize the ROS 2 Node with a unique name
        super().__init__('depth_image_subscriber_node')
        self.get_logger().info("Depth Image Subscriber Node has been started.")

        # Initialize CvBridge
        self.bridge = CvBridge()

        self.sub_disparity = self.create_subscription(
            Image,
            "/simple_drone/camera/depth/image_raw",
            self.get_image,
            10 # QoS queue depth
        )
        self.get_logger().info(f"Subscribing to topic: {self.sub_disparity.topic_name}")

        self.image_count = 0 # Counter for received images

    def get_image(self, msg: Image):
        """
        Callback function for the depth image subscription.

        Args:
            msg (sensor_msgs.msg.Image): The incoming ROS Image message.
        """
        self.image_count += 1
        # self.get_logger().info(f"Received Image No. {self.image_count} on topic: {self.sub_disparity.topic_name}")
        

        try:
            depth_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            normalized_depth_image_display = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            resized_image = cv2.resize(normalized_depth_image_display, (32,32), interpolation=cv2.INTER_LINEAR)
            cv2.imshow('My Image', resized_image)
            cv2.waitKey(1)

            self.get_logger().info(f"  Image Info: height={depth_array.shape[0]}, width={depth_array.shape[1]}")
           
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")


def main(args=None):
    rclpy.init(args=args) 
    
    depth_subscriber = DepthImageSubscriber() 
    
    rclpy.spin(depth_subscriber) 

    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()