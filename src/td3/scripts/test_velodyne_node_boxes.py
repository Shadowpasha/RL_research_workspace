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

import point_cloud2 as pc2
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from gazebo_msgs.srv import SetEntityState,SpawnEntity,DeleteEntity,GetEntityState
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped, Point
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import geometry_msgs.msg

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
TIME_DELTA = 0.2

last_odom = None
environment_dim = 20
velodyne_data = np.ones(environment_dim) * 10

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
            torch.load("%s/%s_actor.pth" % (directory, filename))
        )


class GazeboEnv(Node):
    """Superclass for all Gazebo environments."""

    def __init__(self):
        super().__init__('env')
        self.environment_dim = 20
        self.odom_x = 0
        self.odom_y = 0

        self.goal_x = 1
        self.goal_y = 0.0

        self.upper = 5.0
        self.lower = -5.0

        self.set_self_state = ModelState()
        self.set_self_state.model_name = "r1"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.0
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0

        # Set up the ROS publishers and subscribers
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.set_state = self.create_publisher(ModelState, "gazebo/set_model_state", 10)

        self.unpause = self.create_client(Empty, "/unpause_physics")
        self.pause = self.create_client(Empty, "/pause_physics")
        self.reset_proxy = self.create_client(Empty, "/reset_world")
        self.req = Empty.Request

        self.publisher = self.create_publisher(MarkerArray, "goal_point", 3)
        self.publisher2 = self.create_publisher(MarkerArray, "linear_velocity", 1)
        self.publisher3 = self.create_publisher(MarkerArray, "angular_velocity", 1)

        self.prev_distance = 0.0
        self.box_locations = []
        self.del_model_prox = self.create_client(DeleteEntity,"delete_entity")
        self.spawn_model_client = self.create_client(SpawnEntity,"spawn_entity")

       

    # Perform an action and read a new state
    def step(self, action):
        global velodyne_data
        target = False
        
        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = float(action[0])
        vel_cmd.angular.z = float(action[1])
        self.vel_pub.publish(vel_cmd)
        self.publish_markers(action)

        while not self.unpause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        try:
            self.unpause.call_async(Empty.Request())
        except:
            print("/unpause_physics service call failed")

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        try:
            pass
            self.pause.call_async(Empty.Request())
        except (rclpy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        # read velodyne laser state
        done, collision, min_laser = self.observe_collision(velodyne_data)
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
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
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

        # Detect if the goal has been reached and give a large positive reward
        if distance < GOAL_REACHED_DIST:
            env.get_logger().info("GOAL is reached!")
            target = True
            done = True

        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, collision, action, min_laser, distance)
        return state, reward, done, target

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        #rospy.wait_for_service("/gazebo/reset_world")
        while not self.reset_proxy.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset : service not available, waiting again...')

        try:
            self.reset_proxy.call_async(Empty.Request())
        except rclpy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")

        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        self.clear_boxes()
        time.sleep(2)
        self.randomize_boxes()

        self.odom_x = 0.0
        self.odom_y = 0.0

        # set a random goal in empty space in environment
        self.change_goal()
        # randomly scatter boxes in the environment
        # self.random_box()
        self.publish_markers([0.0, 0.0])

        while not self.unpause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        try:
            self.unpause.call_async(Empty.Request())
        except:
            print("/gazebo/unpause_physics service call failed")

        time.sleep(TIME_DELTA)

        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        try:
            self.pause.call_async(Empty.Request())
        except:
            print("/gazebo/pause_physics service call failed")

        v_state = []
        v_state[:] = velodyne_data[:]
        laser_state = [v_state]

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )
        self.prev_distance = distance

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

    def change_goal(self):
        # Place a new goal and check if its location is not on one of the obstacles
        # if self.upper = 5:
        #     self.upper += 0.004
        # if self.lower > -5:
        #     self.lower -= 0.004

        goal_ok = False

        while not goal_ok:
            self.goal_x = self.odom_x + random.uniform(0.0, 4.5)
            self.goal_y = self.odom_y + random.uniform(-1.5, 1.5)
            goal_ok = self.check_pos_goal(self.goal_x, self.goal_y)

    def check_pos(self,x,y):
        pos_ok = True
        # models = self.model_states
        for model in self.box_locations:
            if( model.position.x + 0.5 > x > model.position.x - 0.5 and model.position.y + 0.5 > y > model.position.y - 0.5):
                pos_ok = False

        if( 0.0 + 1.2 > x > 0.0 - 1.2 and 0.0 + 1.2 > y > 0.0 - 1.2):
            pos_ok = False

        return pos_ok
    
    def check_pos_goal(self,x,y):
        pos_ok = True
        for model in self.box_locations:
            if( model.position.x + 0.7 > x > model.position.x - 0.7 and model.position.y + 0.7 > y > model.position.y - 0.7):
                pos_ok = False
        return pos_ok
    

    def clear_boxes(self):
        for i in range(1,5):
            delete_req = DeleteEntity.Request()
            delete_req.name = "box_" + str(i)
            future = self.del_model_prox.call_async(delete_req)
            if(len(self.box_locations) > 1):
                self.box_locations.pop()
            time.sleep(0.05)
        
        self.del_model_prox.wait_for_service(timeout_sec=1)
        delete_req = DeleteEntity.Request()
        delete_req.name = "goal"
        future = self.del_model_prox.call_async(delete_req)
    
    def randomize_boxes(self):
        
        print("randomizing")
        for i in range(1,5):  # 6 to 12
            box_ok = False
            box_x = 0.0
            box_y = 0.0
            while not box_ok:
                box_x = random.uniform(0.0,3.5)  
                box_y = random.uniform(-2.0,2.0)
                box_ok = self.check_pos(box_x,box_y)
            
            tree = random.randint(0,1)
            if(tree == 0):
                path = "/home/anas/ros2_ws/src/td3/urdf/box.urdf"
            else:
                path = "/home/anas/ros2_ws/src/td3/urdf/wider_box.urdf"

            quaternion = quaternion_from_euler(0.0, 0.0, random.randint(0,360))
            quat = geometry_msgs.msg.Quaternion()
       
            quat.x = quaternion[0]
            quat.y = quaternion[1]
            quat.z = quaternion[2]
            quat.w = quaternion[3]

            spawn_req = SpawnEntity.Request()
            spawn_req.name="box_" + str(i)
            pose = Pose()
            pose.position.x = box_x
            pose.position.y = box_y
            pose.position.z = 0.6
            pose.orientation = quat

            self.box_locations.append(pose)
            spawn_req.initial_pose = pose
            spawn_req.robot_namespace='/box'
            spawn_req.reference_frame = 'world'
            spawn_req.xml = open(path,'r').read()

            future = self.spawn_model_client.call_async(spawn_req)

        time.sleep(2.0)

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

    @staticmethod
    def observe_collision(laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        if min_laser < COLLISION_DIST:
            env.get_logger().info("Collision is detected!")
            return True, True, min_laser
        return False, False, min_laser

    def get_reward(self, target, collision, action, min_laser, distance):
        if target:
            env.get_logger().info("reward 100")
            return 100.0
        elif collision:
            env.get_logger().info("reward -100")
            return -100.0
        else:
            # r3 = lambda x: 1 - x if x < 1 else 0.0
        
            costs = (self.prev_distance - distance) * 10
            self.prev_distance = distance
            # env.get_logger().info(str(costs))
            return costs 
            # return action[0] / 2 - abs(action[1]) / 2 - r3(min_laser) / 2

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
            if data[i][2] > -0.2:
                dot = data[i][0] * 1 + data[i][1] * 0
                mag1 = math.sqrt(math.pow(data[i][0], 2) + math.pow(data[i][1], 2))
                mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
                beta = math.acos(dot / (mag1 * mag2)) * np.sign(data[i][1])
                dist = math.sqrt(data[i][0] ** 2 + data[i][1] ** 2 + data[i][2] ** 2)

                for j in range(len(self.gaps)):
                    if self.gaps[j][0] <= beta < self.gaps[j][1]:
                        velodyne_data[j] = min(velodyne_data[j], dist)
                        break

if __name__ == '__main__':

    rclpy.init(args=None)

    # Set the parameters for the implementation
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # cuda or cpu
    seed = 0  # Random seed number
    max_ep = 500  # maximum number of steps per episode
    file_name = "td3_velodyne_distance_boxes25-07-19-11-12-52"  # name of the file to load the policy from
    environment_dim = 20
    robot_dim = 4

    torch.manual_seed(seed)
    np.random.seed(seed)
    state_dim = environment_dim + robot_dim
    action_dim = 2

    # Create the network
    network = td3(state_dim, action_dim)
    try:
        network.load(file_name, "/home/anas/ros2_ws/src/td3/scripts/pytorch_models")
    except:
        raise ValueError("Could not load the stored model parameters")

    done = True
    episode_timesteps = 0

    # Create the testing environment
    env = GazeboEnv()
    odom_subscriber = Odom_subscriber()
    velodyne_subscriber = Velodyne_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(odom_subscriber)
    executor.add_node(velodyne_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    rate = odom_subscriber.create_rate(2)

    # Begin the testing loop
    while rclpy.ok():

        # On termination of episode
        if done:
            state = env.reset()

            action = network.get_action(np.array(state))
            # Update action to fall in range [0,1] for linear velocity and [-1,1] for angular velocity
            a_in = [(action[0] + 1) / 2, action[1]]
            next_state, reward, done, target = env.step(a_in)
            print(next_state)
            done = 1 if episode_timesteps + 1 == max_ep else int(done)

            done = False
            episode_timesteps = 0
        else:
            action = network.get_action(np.array(state))
            # Update action to fall in range [0,1] for linear velocity and [-1,1] for angular velocity
            a_in = [(action[0] + 1) / 2, action[1]]
            next_state, reward, done, target = env.step(a_in)
            done = 1 if episode_timesteps + 1 == max_ep else int(done)

            state = next_state
            episode_timesteps += 1

    rclpy.shutdown()
