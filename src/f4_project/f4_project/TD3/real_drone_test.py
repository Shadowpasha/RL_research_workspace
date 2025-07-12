import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
import time
from rclpy.executors import MultiThreadedExecutor
import threading
import argparse
import numpy as np
import TD3
import math
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, TwistStamped, Point
from sensor_msgs.msg import LaserScan

node = rclpy.create_node('Takoff_command')
executor = MultiThreadedExecutor()
pose = Pose()
laser_ranges = np.zeros(10)
laser_ranges_360 = np.zeros(20)
goal_reached=False
goal_heading = 0.0
distance = 99.0
yaw = 0.0
goal = [0.0,0.5]
action = [0.0, 0.0]


def node_spin_fun():
    global executor
    executor.spin()

def position_cb(msg):
        global pose, goal, goal_reached, yaw, distance, goal_heading
        pose = msg.pose
        orientation_q = [0.0,0.0,0.0,0.0]
        orientation_q[0] = pose.orientation.x
        orientation_q[1] = pose.orientation.y
        orientation_q[2] = pose.orientation.z
        orientation_q[3] = pose.orientation.w
        _, _, yaw = euler_from_quaternion(orientation_q)
        distance = math.sqrt(math.pow((goal[0] - pose.position.x),2) + math.pow((goal[1] - pose.position.y),2))
        goal_heading = math.atan2((goal[1] - pose.position.y), goal[0]- pose.position.x)
        
        if(abs(distance) < 0.5):
            goal_reached = True

def get_laser(msg):
        global laser_ranges
        laser_ranges_temp = msg.ranges
        final_laser_range = []
        final_laser_range = np.array(laser_ranges_temp)
        final_laser_range[np.isnan(final_laser_range)] = 5.0
        final_laser_range[np.isinf(final_laser_range)] = 0.2
        final_laser_range = final_laser_range/5.0
        laser_ranges = final_laser_range

def get_laser_360(msg):
        global laser_ranges_360
        laser_ranges_temp = msg.ranges
        final_laser_range = []
        final_laser_range = np.array(laser_ranges_temp)
        final_laser_range[np.isinf(final_laser_range)] = 5.0
        final_laser_range = final_laser_range/5.0
        laser_ranges_360 = final_laser_range

def main(args=None):
    global node, executor
    executor.add_node(node)
    et = threading.Thread(target=node_spin_fun)
    et.start() 

    qos_profile_laser = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.SYSTEM_DEFAULT,
            history=HistoryPolicy.SYSTEM_DEFAULT,
            depth=10
        )

    qos_profile_drone = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
    
    rclpy.init(args=args)
    publisher_twist = node.create_publisher(Twist, 'cmd_vel', 10)
    publisher_mode = node.create_publisher(String, 'change_mode', 10)
    pos_sub = node.create_subscription(PoseStamped,"/mavros/local_position/pose",position_cb, qos_profile_drone)
    sub_disparity_360  = node.create_subscription(LaserScan,"/scan",get_laser_360, qos_profile_laser)
    sub_disparity  = node.create_subscription(LaserScan,"/realsense_scan",get_laser, qos_profile_laser)

    position = Twist()
    mode = String()
    state = "Take off"
    parser = argparse.ArgumentParser()
    target_position_x = 0.0
    target_position_yaw = 0.0
    parser.add_argument("--load_model", default="2d_new_28_10_2024_09_36") 
    args = parser.parse_args()

    kwargs = {
        "state_dim": 34,
        "action_dim": 2,
        "max_action": 1,
        "discount": 0.99,
        "tau": 0.005,
    }
    policy = TD3.TD3(**kwargs)

    while rclpy.ok():

        if(state == "Take off"):
            position.linear.x = 0.0
            position.linear.y = 0.0
            position.linear.z = 0.5
            position.angular.z = 0.0
            publisher_twist.publish(position)
            mode.data = "takeoff"
            publisher_mode.publish(mode)
            time.sleep(3)
            state = "Navigate"

        elif(state == "Navigate"):
            mode.data = "position"
            publisher_mode.publish(mode)

            laser_combination_level = np.append(laser_ranges, laser_ranges_360)
            goal_data = np.array([action[0],action[1], distance, goal_heading - yaw])
            state =  np.append(laser_combination_level, goal_data)
            action = policy.select_action(state,[])

            target_position_x += float(((action[0])+ 1.0)*0.025)
            target_position_yaw += float((action[1]))*0.05

            vel_cmd = Twist()
            vel_cmd.linear.x = target_position_x
            vel_cmd.linear.y = 0.0
            vel_cmd.linear.z = 0.0
            vel_cmd.angular.z = target_position_yaw

            publisher_twist.publish(vel_cmd)

            time.sleep(0.2)

            if(goal_reached):
                state = "Land"
            
        elif(state == "Land"):
            mode.data = "land"
            publisher_mode.publish(mode)
            state="End"
                
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()