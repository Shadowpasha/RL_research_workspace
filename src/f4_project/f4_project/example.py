#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from gazebo_msgs.msg import ContactsState,ModelStates

qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )

rclpy.init()
node = rclpy.create_node("training")


def get_model_states(msg):
    print("here")

states_sub = node.create_subscription(ModelStates,'model_states',get_model_states, 10)


while rclpy.ok():
        rclpy.spin_once(node)

