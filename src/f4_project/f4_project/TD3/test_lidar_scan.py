import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3, PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import time
from rclpy.executors import MultiThreadedExecutor
import threading
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
import math


qos_profile_laser = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.SYSTEM_DEFAULT,
            history=DurabilityPolicy.SYSTEM_DEFAULT,
            depth=10
        )

lidar_laser_ranges = []

def get_laser(msg):
    global lidar_laser_ranges
    lidar_laser_ranges = []
    final_laser_range = np.array(msg.ranges)[150:-150]
    final_laser_range[np.isnan(final_laser_range)] = 5.0
    final_laser_range[np.isinf(final_laser_range)] = 0.2
    final_laser_range = final_laser_range/5.0
    for i in range(len(final_laser_range)):
        if(i % 10 == 0):
            lidar_laser_ranges.append(final_laser_range[i])


rclpy.init()
node = rclpy.create_node('use_rl')
executor = MultiThreadedExecutor()
laser_sub  = node.create_subscription(LaserScan,"/scan", get_laser, qos_profile_laser)

def node_spin_fun():
    global executor
    executor.spin()



executor.add_node(node)
et = threading.Thread(target=node_spin_fun)
et.start() 

time.sleep(1)

while True:
    print("Laser Range:")
    # print(len(lidar_laser_ranges))
    print(lidar_laser_ranges)
    time.sleep(0.5)
        
executor.shutdown()
node.destroy_node()
rclpy.shutdown()