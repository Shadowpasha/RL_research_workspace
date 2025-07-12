import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3, PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
import time
from rclpy.executors import MultiThreadedExecutor
import threading

node = rclpy.create_node('Takoff_command')
executor = MultiThreadedExecutor()

def node_spin_fun():
    global executor
    executor.spin()

def main(args=None):
    global node, executor

    node = rclpy.create_node('minimal_publisher')

    publisher_twist = node.create_publisher(Twist, 'cmd_vel', 10)
    publisher_mode = node.create_publisher(String, 'change_mode', 10)

    executor.add_node(node)
    et = threading.Thread(target=node_spin_fun)
    et.start()
    time.sleep(1)


    position = Twist()
    mode = String()
    state = "Take off"

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
            state = "Forward"
                

        elif(state == "Forward"):
            mode.data = "position"
            publisher_mode.publish(mode)
            position.linear.x = 1.0
            position.linear.y = 0.0
            position.linear.z = 0.5
            position.angular.z = 0.0
            publisher_twist.publish(position)
            time.sleep(5) 
            state = "Left"


        elif(state == "Left"):
            position.linear.x = 1.0
            position.linear.y = 1.0
            position.linear.z = 1.2
            position.angular.z = 0.0
            publisher_twist.publish(position)
            time.sleep(5) 
            state = "Backward"


        elif(state == "Backward"):
            position.linear.x = 0.0
            position.linear.y = 1.0
            position.linear.z = 1.2
            position.angular.z = 0.0
            publisher_twist.publish(position)
            time.sleep(5) 
            state = "Right"


        elif(state == "Right"):
            position.linear.x = 0.0
            position.linear.y = 0.0
            position.linear.z = 1.2
            position.angular.z = 0.0
            publisher_twist.publish(position)
            time.sleep(5) 
            state = "Land"

        elif(state == "Land"):
            mode.data = "land"
            publisher_mode.publish(mode)
            state="End"

    node.destroy_node()
    rclpy.shutdown()