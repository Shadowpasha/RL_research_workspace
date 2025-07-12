import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3, PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import time
from rclpy.executors import MultiThreadedExecutor
import threading
import TD3
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
import math

state_dim = 14
action_dim = 2
max_action = 1.0

kwargs = {
		"state_dim": state_dim,
		"action_dim": action_dim,
		"max_action": max_action,
		"discount": 0.99,
		"tau": 0.005,
	}
kwargs["policy_noise"] = 0.2 * max_action
kwargs["noise_clip"] = 0.5 * max_action
kwargs["policy_freq"] = 2
policy = TD3.TD3(**kwargs)
policy.load(f"models/Tier_1_05_03_2025_12_02")

qos_profile_laser = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.SYSTEM_DEFAULT,
            history=DurabilityPolicy.SYSTEM_DEFAULT,
            depth=10
        )

qos_profile_drone = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

lidar_laser_ranges = []
distance = 0.0
goal_heading = 0.0
goal = [1.0, 0.0]
trueYaw = 0.0

def get_laser(self,msg):
    global lidar_laser_ranges
    lidar_laser_ranges = []
    final_laser_range = np.array(msg.ranges)[150:-150]
    final_laser_range[np.isnan(final_laser_range)] = 5.0
    final_laser_range[np.isinf(final_laser_range)] = 0.2
    final_laser_range = final_laser_range/5.0
    for i in range(len(final_laser_range)):
        if(i % 10 == 0):
            lidar_laser_ranges.append(final_laser_range[i])

def position_cb(self,msg):
        global distance, goal_heading, goal, trueYaw
        pose = msg.pose
        orientation_q = [0.0,0.0,0.0,0.0]
        orientation_q[0] = pose.orientation.x
        orientation_q[1] = pose.orientation.y
        orientation_q[2] = pose.orientation.z
        orientation_q[3] = pose.orientation.w

        #trueYaw is the drones current yaw value
        pitch, roll, trueYaw = euler_from_quaternion(orientation_q)
        distance = math.sqrt(math.pow((goal[0] - pose.position.x),2) + math.pow((goal[1] - pose.position.y),2))
        goal_heading = math.atan2((goal[1] - pose.position.y), goal[0]-pose.position.x)

rclpy.init()
node = rclpy.create_node('use_rl')
executor = MultiThreadedExecutor()
laser_sub  = node.create_subscription(LaserScan,"/scan", get_laser, qos_profile_laser)
pos_sub = node.create_subscription(PoseStamped,"/mavros/local_position/pose", position_cb, qos_profile_drone)

def node_spin_fun():
    global executor
    executor.spin()

def main(args=None):
    global node, executor, distance, goal_heading, trueYaw
    publisher_twist = node.create_publisher(Twist, 'cmd_vel', 10)
    publisher_mode = node.create_publisher(String, 'change_mode', 10)
    executor.add_node(node)
    et = threading.Thread(target=node_spin_fun)
    et.start() 
    
    time.sleep(1)
    position = Twist()
    mode = String()
    state = "Take off"

    done = False

    while True:
        if(state == "Take off"):
            position.linear.x = 0.0
            position.linear.y = 0.0
            position.linear.z = 0.5
            position.angular.z = 0.0
            publisher_twist.publish(position)
            mode.data = "takeoff"
            publisher_mode.publish(mode)
            time.sleep(2)
            state = "Navigate"

        elif(state == "Navigate"):
            mode.data = "position"
            publisher_mode.publish(mode)
            goal_data = np.array([0.0, 0.0, distance, goal_heading - trueYaw])
            state = np.append(lidar_laser_ranges, goal_data)

            while (not done):
                action = policy.select_action(state)

                vel_cmd = Twist()
                # vel_cmd.linear.x = (action[0] + 1.01) * 0.05
                vel_cmd.linear.x = float(((action[0])+ 1.0)*0.025)
                vel_cmd.linear.y = 0.0
                vel_cmd.linear.z = 0.0
                vel_cmd.angular.z = float((action[1]))*0.05
                publisher_twist.publish(vel_cmd)

                time.sleep(0.1)

                goal_data = np.array([action[0], action[1], distance, goal_heading - trueYaw])
                state = np.append(lidar_laser_ranges, goal_data)

                if(distance < 0.3):
                    break
            
            state = "Land"
            
        elif(state == "Land"):
            mode.data = "land"
            publisher_mode.publish(mode)
            state="End"
                
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()