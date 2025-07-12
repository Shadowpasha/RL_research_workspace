import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3, PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
import time
from rclpy.executors import MultiThreadedExecutor
import threading

rclpy.init()
node = rclpy.create_node('Takoff_command')
executor = MultiThreadedExecutor()

def node_spin_fun():
    global executor
    executor.spin()

def main(args=None):
    global node, executor
    publisher_twist = node.create_publisher(Twist, 'cmd_vel', 10)
    publisher_mode = node.create_publisher(String, 'change_mode', 10)
    executor.add_node(node)
    et = threading.Thread(target=node_spin_fun)
    et.start() 
    
    time.sleep(1)
    position = Twist()
    mode = String()
    state = "Take off"

    while True:
        if(state == "Take off"):
            position.linear.x = 0.0
            position.linear.y = 0.0
            position.linear.z = 0.75
            position.angular.z = 0.0
            publisher_twist.publish(position)
            mode.data = "takeoff"
            publisher_mode.publish(mode)
            time.sleep(2)
            state = "Standby"

        elif(state == "Standby"):
            mode.data = "position"
            publisher_mode.publish(mode)
            time.sleep(3)
            state = "Land"
            
        elif(state == "Land"):
            mode.data = "land"
            publisher_mode.publish(mode)
            state="End"
                
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()