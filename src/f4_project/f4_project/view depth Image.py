import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
import numpy as np

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        bridge = CvBridge()
        dataimg = bridge.imgmsg_to_cv2(msg,"32FC1")
        dataimg = cv.normalize(dataimg, dataimg, 0, 255, cv.NORM_MINMAX)
        dataimg = np.array(dataimg, dtype = np.uint8)
        dataimg = dataimg[120:360,0:848]
        # print(dataimg.shape)
        cv.imshow("window",dataimg)
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()