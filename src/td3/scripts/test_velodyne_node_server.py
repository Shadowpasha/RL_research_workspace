#!/usr/bin/env python3

import time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

import rclpy
from rclpy.node import Node
import threading

import math
import random
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
import point_cloud2 as pc2
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from squaternion import Quaternion
from std_srvs.srv import Empty
from std_msgs.msg import Empty as EmptyMsg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan

GOAL_REACHED_DIST = 0.15
COLLISION_DIST = 0.2
FAIL_SAFE_DIST = 0.5  # Activate fail-safe if obstacle is closer than this
FAIL_SAFE_RELEASE_DIST = 0.6 # Release fail-safe if obstacle is further than this (hysteresis)
TIME_DELTA = 0.05

last_odom = None
environment_dim = 20
velodyne_data = np.ones(environment_dim) * 10
laser_data = np.ones(environment_dim) * 10 # This variable seems unused, consider removing or using

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()

        self.layer_1 = nn.Linear(state_dim, 800)
        self.layer_2 = nn.Linear(800, 600)
        self.layer_3 = nn.Linear(600, action_dim)
        self.tanh = nn.Tanh()

    def forward(self, s):
        s = F.relu(self.layer_1(s))
        s = F.relu(self.layer_2(s))
        a = self.tanh(self.layer_3(s))
        return a

# td3 network
class td3(object):
    def __init__(self, state_dim, action_dim):
        # Initialize the Actor network
        self.actor = Actor(state_dim, action_dim).to(device)

    def get_action(self, state):
        # Function to get the action from the actor
        state = torch.Tensor(state.reshape(1, -1)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()

    def load(self, filename, directory):
        # Function to load network parameters
        self.actor.load_state_dict(
            torch.load("%s/%s_actor.pth" % (directory, filename),weights_only=True)
        )

class GazeboEnv(Node):
    """Superclass for all Gazebo environments."""

    def __init__(self):
        super().__init__('env')

        self.environment_dim = 20
        self.odom_x = 0
        self.odom_y = 0

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.colilision_counter = 0
        self.current_goal_active = False
        self.fail_safe_active = False # New flag for fail-safe state

        # Set up the ROS publishers and subscribers
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.publisher = self.create_publisher(MarkerArray, "goal_point", 3)
        self.publisher2 = self.create_publisher(MarkerArray, "linear_velocity", 1)
        self.publisher3 = self.create_publisher(MarkerArray, "angular_velocity", 1)

        qos_profile_commands = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1, # No need for a deep history for simple commands like this
            durability=DurabilityPolicy.TRANSIENT_LOCAL # Keep this volatile to match if you prefer. Or make both transient local.
                                                 # For a simple empty message, VOLATILE is often sufficient IF both are running.
        )

        # New subscribers for goal and cancellation
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_callback,
            10
        )
        self.cancel_subscriber = self.create_subscription(
            EmptyMsg,
            "/cancel_navigation",
            self.cancel_callback,
            qos_profile_commands
        )

        # New publisher for navigation status
        self.done_navigating_pub = self.create_publisher(EmptyMsg, "/done_navigating", 1)

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.current_goal_active = True
        self.fail_safe_active = False # Reset fail-safe on new goal
        self.get_logger().info(f"New goal received: ({self.goal_x}, {self.goal_y}). Navigation started.")
        self.colilision_counter = 0

    def cancel_callback(self, msg):
        if self.current_goal_active:
            self.current_goal_active = False
            self.fail_safe_active = False # Reset fail-safe on cancellation
            self.stop_robot()
            self.get_logger().info("Navigation goal cancelled.")
            self.done_navigating_pub.publish(EmptyMsg())

    def stop_robot(self):
        """
        Publishes a zero-velocity command to stop the robot.
        This is called during node shutdown or goal cancellation/completion.
        """
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        self.vel_pub.publish(vel_cmd)
        time.sleep(0.1)
        self.fail_safe_active = False # Ensure fail-safe is off when stopped

    def step(self, action_rl): # Renamed action to action_rl to distinguish from fail-safe action
        global velodyne_data
        target = False
        done = False # Initialize done here

        # read velodyne laser state
        done_collision, collision, min_laser = self.observe_collision(velodyne_data)
        v_state = []
        v_state[:] = velodyne_data[:]
        laser_state = [v_state]

        # Calculate robot heading from odometry data
        self.odom_x = last_odom.pose.pose.position.x
        self.odom_y = last_odom.pose.pose.position.y
        quaternion = Quaternion(
            last_odom.pose.pose.orientation.w,
            last_odom.pose.pose.orientation.x,
            last_odom.pose.pose.orientation.y,
            last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        distance = np.linalg.norm(
            [self.goal_x - self.odom_x, self.goal_y - self.odom_y]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Fail-safe logic
        action_to_publish = action_rl # Default to RL action
        if min_laser < FAIL_SAFE_DIST and not self.fail_safe_active:
            self.get_logger().warn(f"Fail-safe activated! Obstacle at {min_laser:.2f}m.")
            self.fail_safe_active = True
        elif self.fail_safe_active and min_laser > FAIL_SAFE_RELEASE_DIST:
            self.get_logger().info(f"Fail-safe released. Obstacle at {min_laser:.2f}m.")
            self.fail_safe_active = False

        if self.fail_safe_active:
            # Simple fail-safe: turn away from the obstacle or back up
            vel_cmd = Twist()
            # Determine which side the obstacle is closer and turn away
            # We assume velodyne_data is sorted angularly.
            # Find the index of the minimum laser reading to infer direction.
            min_laser_idx = np.argmin(velodyne_data)
            
            # Divide the 20 segments into left and right (arbitrary threshold)
            # This is a very basic heuristic. A better approach would map angles.
            if min_laser_idx < environment_dim / 2: # Obstacle likely on the right side of robot
                vel_cmd.angular.z = -0.5 # Turn left
                vel_cmd.linear.x = 0.2 # Stop linear movement
            else: # Obstacle likely on the left side of robot
                vel_cmd.angular.z = 0.5 # Turn right
                vel_cmd.linear.x = 0.2 # Stop linear movement

            # If obstacle is directly in front, consider backing up
            if min_laser < (COLLISION_DIST + 0.1) and (min_laser_idx in [8, 9, 10, 11]): # Example indices for front
                 vel_cmd.linear.x = -0.1 # Move backward slowly
                 vel_cmd.angular.z = 0.0 # Prioritize backward movement
            
            action_to_publish = [vel_cmd.linear.x, vel_cmd.angular.z]
            self.get_logger().info(f"Fail-safe action: Linear={action_to_publish[0]:.2f}, Angular={action_to_publish[1]:.2f}")

        # Publish the robot action (either RL or fail-safe)
        vel_cmd = Twist()
        vel_cmd.linear.x = float(action_to_publish[0])
        vel_cmd.angular.z = float(action_to_publish[1])
        self.vel_pub.publish(vel_cmd)
        self.publish_markers(action_to_publish) # Use the action that was actually published

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        # Detect if the goal has been reached and give a large positive reward
        if distance < GOAL_REACHED_DIST:
            env.get_logger().info("GOAL is reached!")
            target = True
            done = True
            self.current_goal_active = False
            self.stop_robot()
            self.done_navigating_pub.publish(EmptyMsg())

        if collision: # This 'collision' comes from observe_collision
            self.current_goal_active = False
            done = True # Mark done if collision
            self.stop_robot()
            self.done_navigating_pub.publish(EmptyMsg())

        robot_state = [distance, theta, action_to_publish[0], action_to_publish[1]] # Use published action
        state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, collision, action_to_publish, min_laser) # Use published action for reward

        return state, reward, done, target

    def reset(self):
        # Resets the state of the environment and returns an initial observation.
        # This reset is intended for the start of a new navigation attempt.
        if not self.current_goal_active:
            # If no goal is active, we don't need to reset the robot state for navigation.
            # We can return a default state or wait for a goal.
            return np.zeros(self.environment_dim + 4) # Return a zero state

        quaternion = Quaternion(
            last_odom.pose.pose.orientation.w,
            last_odom.pose.pose.orientation.x,
            last_odom.pose.pose.orientation.y,
            last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        self.odom_x = last_odom.pose.pose.position.x
        self.odom_y = last_odom.pose.pose.position.y

        self.publish_markers([0.0, 0.0])

        time.sleep(TIME_DELTA)

        v_state = []
        v_state[:] = velodyne_data[:]
        laser_state = [v_state]

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        robot_state = [distance, theta, 0.0, 0.0]
        state = np.append(laser_state, robot_state)
        return state

    def publish_markers(self, action):
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0.0

        markerArray.markers.append(marker)

        self.publisher.publish(markerArray)

        markerArray2 = MarkerArray()
        marker2 = Marker()
        marker2.header.frame_id = "odom"
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.scale.x = float(abs(action[0]))
        marker2.scale.y = 0.1
        marker2.scale.z = 0.01
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 5.0
        marker2.pose.position.y = 0.0
        marker2.pose.position.z = 0.0

        markerArray2.markers.append(marker2)
        self.publisher2.publish(markerArray2)

        markerArray3 = MarkerArray()
        marker3 = Marker()
        marker3.header.frame_id = "odom"
        marker3.type = marker.CUBE
        marker3.action = marker.ADD
        marker3.scale.x = float(abs(action[1]))
        marker3.scale.y = 0.1
        marker3.scale.z = 0.01
        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.pose.position.x = 5.0
        marker3.pose.position.y = 0.2
        marker3.pose.position.z = 0.0

        markerArray3.markers.append(marker3)
        self.publisher3.publish(markerArray3)

    def observe_collision(self, laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        if min_laser < COLLISION_DIST:
            self.colilision_counter +=1
            if(self.colilision_counter > 5): # Require 5 consecutive readings below COLLISION_DIST to confirm collision
                env.get_logger().info("Collision is detected!")
                return True, True, min_laser # done, collision, min_laser
        else:
            self.colilision_counter = 0 # Reset counter if not in collision
        return False, False, min_laser # done, collision, min_laser

    @staticmethod
    def get_reward(target, collision, action, min_laser):
        if target:
            env.get_logger().info("reward 100")
            return 100.0
        elif collision:
            env.get_logger().info("reward -100")
            return -100.0
        else:
            r3 = lambda x: 1 - x if x < 1 else 0.0
            return action[0] / 2 - abs(action[1]) / 2 - r3(min_laser) / 2

class Odom_subscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription

    def odom_callback(self, od_data):
        global last_odom
        last_odom = od_data

class Velodyne_subscriber(Node):

    def __init__(self):
        super().__init__('velodyne_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            "/velodyne_points",
            self.velodyne_callback,
            10)
        self.subscription

        self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / environment_dim]]
        for m in range(environment_dim - 1):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / environment_dim]
            )
        self.gaps[-1][-1] += 0.03

    def velodyne_callback(self, v):
        global velodyne_data
        data = list(pc2.read_points(v, skip_nans=False, field_names=("x", "y", "z")))
        velodyne_data = np.ones(environment_dim) * 10
        for i in range(len(data)):
            if data[i][2] > -0.2: # Filter out ground points
                # Calculate angle of the point relative to robot's forward direction
                dot = data[i][0] * 1 + data[i][1] * 0
                mag1 = math.sqrt(math.pow(data[i][0], 2) + math.pow(data[i][1], 2))
                mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
                if mag1 == 0: # Avoid division by zero
                    continue
                beta = math.acos(dot / (mag1 * mag2)) * np.sign(data[i][1])
                dist = math.sqrt(data[i][0] ** 2 + data[i][1] ** 2 + data[i][2] ** 2)

                for j in range(len(self.gaps)):
                    if self.gaps[j][0] <= beta < self.gaps[j][1]:
                        velodyne_data[j] = min(velodyne_data[j], dist)
                        break

if __name__ == '__main__':

    rclpy.init(args=None)

    # Set the parameters for the implementation
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    seed = 0
    max_ep = 9999999999
    file_name = "td3_velodyne_distance_boxes_unlimited25-07-23-21-26"
    environment_dim = 20
    robot_dim = 4

    torch.manual_seed(seed)
    np.random.seed(seed)
    state_dim = environment_dim + robot_dim
    action_dim = 2

    # # Create the network
    network = td3(state_dim, action_dim)
    try:
        network.load(file_name, "/home/anas/ros2_ws/src/td3/scripts/pytorch_models")
    except Exception as e:
        raise ValueError(f"Could not load the stored model parameters: {e}")

    done = False
    episode_timesteps = 0

    # Create the testing environment
    env = GazeboEnv()
    odom_subscriber = Odom_subscriber()
    velodyne_subscriber = Velodyne_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odom_subscriber)
    executor.add_node(velodyne_subscriber)
    executor.add_node(env)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    time.sleep(1.5)
    rate = odom_subscriber.create_rate(4)

    env.get_logger().info("Waiting for a goal pose on /goal_pose...")

    try:
        while rclpy.ok():
            if env.current_goal_active:
                if done:
                    env.get_logger().info("Navigation finished for the current goal. Waiting for a new one.")
                    done = False
                    episode_timesteps = 0

                state = env.reset()

                if last_odom is None:
                    env.get_logger().info("Waiting for odometry data...")
                    time.sleep(0.5)
                    continue

                # Get action from RL model, but it might be overridden by fail-safe in env.step
                action_rl = network.get_action(np.array(state))
                # Update action to fall in range [0,1] for linear velocity and [-1,1] for angular velocity
                a_in = [((action_rl[0]*1.0 + 1) / 2.0), (action_rl[1])]

                next_state, reward, done, target = env.step(a_in) # Pass RL action

                done = done or (episode_timesteps + 1 == max_ep)

                state = next_state
                episode_timesteps += 1
            else:
                env.stop_robot()
                episode_timesteps = 0
                done = False
                # rclpy.spin_once(env, timeout_sec=0.5)

    except KeyboardInterrupt:
        env.get_logger().info("KeyboardInterrupt received. Initiating shutdown.")
        env.stop_robot()
        rclpy.shutdown()