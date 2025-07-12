#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Quaternion
from math import pi
from std_msgs.msg import Bool,String, Empty
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandHome
from geographic_msgs.msg import GeoPointStamped
import time
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import termios
import tty
import sys
from select import select


class OffboardControl(Node):

    def __init__(self):
        super().__init__('Position_control_Guided')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        #Create subscriptions
        self.status_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.vehicle_status_callback,
            qos_profile)
        
        self.offboard_velocity_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.offboard_position_callback,
            10)
        
        self.control_sub = self.create_subscription(
            String,
            '/change_mode',
            self.change_mode,
            10)
        
        self.emergency_sub = self.create_subscription(
            Empty,
            '/emergency',
            self.stop_all,
            10)
        
        self.attitude_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.attitude_callback,
            qos_profile)
        
        # self.my_bool_sub = self.create_subscription(
        #     Bool,
        #     '/arm_message',
        #     self.arm_message_callback,
        #     qos_profile)


        #Create publishers
        # self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        # self.publisher_velocity = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)
        self.arm_service = self.create_client(CommandBool, "mavros/cmd/arming")
        self.set_origin = self.create_publisher(GeoPointStamped, "/mavros/global_position/set_gp_origin", qos_profile)

        self.mode_service = self.create_client(SetMode, "mavros/set_mode")
        self.takeoff_service = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.done_pub = self.create_publisher(String, "position_reached",qos_profile=1)
        # self.position_status = "Going"
        
        # self.settings = termios.tcgetattr(sys.stdin)
        # self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        self.x_val = 0.0
        self.y_val = 0.0
        self.z_val = 0.0
        self.yaw_val = 0
        self.yaw_init = False
        self.takeoff_status = "none"
        self.current_state = State()
        self.offb_set_mode = SetMode.Request()
        self.offb_set_mode.custom_mode = 'GUIDED'

        self.arm_cmd = CommandBool.Request()
        self.arm_cmd.value = True

        self.takeoff_cmd = CommandTOL.Request()
        self.takeoff_cmd.latitude = 0.0
        self.takeoff_cmd.longitude = 0.0
        self.takeoff_cmd.min_pitch = 0.0
        self.takeoff_cmd.altitude = 0.5
        self.takeoff_cmd.yaw = 0.0

        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = 0.4 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.velocity = Vector3()
        self.yaw = 0.0  #yaw value we send as command
        self.trueYaw = 0.0  #current yaw value of drone
        self.mode = "position"
        self.intial_yaw = 0.0

        self.origin_msg = GeoPointStamped()
        self.origin_msg.position.latitude = 1.5631
        self.origin_msg.position.longitude = 103.645
        self.origin_msg.position.altitude = 0.0
        self.origin_msg.header.stamp = self.get_clock().now().to_msg()
        self.origin_msg.header.frame_id = "uas1"

        for i in range(20):
             self.set_origin.publish(self.origin_msg)
             time.sleep(0.2)

    def change_mode(self,msg):
        self.mode = msg.data
        # print(self.mode)

    def stop_all(self, msg):
        self.mode = "abort"

    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):

        if(self.mode == "takeoff"):
            if(self.takeoff_status == "none"):
                self.takeoff_cmd.altitude = self.z_val
                self.takeoff_service.call_async(self.takeoff_cmd)
                self.takeoff_status = "taking_off"

                self.get_logger().info("TAKEOFF")
        elif(self.mode == "position"):
            self.takeoff_status = "none"
            if(self.current_state.mode != "GUIDED"):
                self.offb_set_mode.custom_mode = 'GUIDED'

                reply = self.mode_service.call_async(self.offb_set_mode)
                self.get_logger().info("GUIDED")

            elif(self.current_state.mode == "GUIDED" and self.current_state.armed == False):
                self.arm_cmd.value = True
                self.arm_service.call_async(self.arm_cmd)
                self.get_logger().info("ARMED")

        elif(self.mode == "land"):
            self.takeoff_status = "none"
            if(self.current_state.mode  != "LAND"):
                self.offb_set_mode.custom_mode = 'LAND'
                reply = self.mode_service.call_async(self.offb_set_mode)
                self.get_logger().info("LAND")

        elif(self.mode == "abort"):
            self.takeoff_status = "none"
            if self.current_state.armed == True:
                self.arm_cmd.value = False
                self.arm_service.call_async(self.arm_cmd)
                self.get_logger().info("DISARMED")

            if(self.current_state.mode  != "LAND"):
                self.offb_set_mode.custom_mode = 'LAND'
                reply = self.mode_service.call_async(self.offb_set_mode)
                self.get_logger().info("LAND")

            
    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):
        self.current_state = msg

    #receives Twist commands from Teleop and converts NED -> FLU
    def offboard_position_callback(self, msg):
        cos_yaw = np.cos(self.trueYaw)
        sin_yaw = np.sin(self.trueYaw)
        
        self.x_val = (msg.linear.x * cos_yaw - msg.linear.y * sin_yaw)
       
        self.y_val = (msg.linear.x * sin_yaw + msg.linear.y * cos_yaw)

        self.z_val = msg.linear.z

        self.yaw_val = self.intial_yaw + msg.angular.z 

        self.position_status = "Going"

    #receives current trajectory values from drone and grabs the yaw value of the orientation
    def attitude_callback(self, msg):
        orientation_q = [0.0,0.0,0.0,0.0]
        orientation_q[0] = msg.pose.orientation.x
        orientation_q[1] = msg.pose.orientation.y
        orientation_q[2] = msg.pose.orientation.z
        orientation_q[3] = msg.pose.orientation.w

        #trueYaw is the drones current yaw value
        pitch, roll, self.trueYaw = euler_from_quaternion(orientation_q)

        if(self.intial_yaw == 0.0 and self.current_state.armed == True):
            self.intial_yaw = self.trueYaw
            self.yaw_val = self.intial_yaw
            self.yaw_init = True
            print("Yaw Initialized")
            # print(self.intial_yaw)
                                        
        
    #publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if(self.mode == "position" and self.yaw_init == True):
            # Publish offboard control modes

            offboard_msg = PoseStamped()
       
            offboard_msg.pose.position.x = self.x_val
            offboard_msg.pose.position.y = self.y_val
            offboard_msg.pose.position.z = self.z_val

            quaternion = quaternion_from_euler(0.0,0.0,self.yaw_val)
            quat = Quaternion()
       
            quat.x = quaternion[0]
            quat.y = quaternion[1]
            quat.z = quaternion[2]
            quat.w = quaternion[3]
            
            offboard_msg.pose.orientation =  quat
            # print(offboard_msg)
            self.publisher_trajectory.publish(offboard_msg)
    

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()