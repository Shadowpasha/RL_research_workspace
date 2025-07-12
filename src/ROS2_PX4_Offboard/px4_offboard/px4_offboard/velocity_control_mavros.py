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
from std_msgs.msg import Bool,String
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import time
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
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
            self.offboard_velocity_callback,
            1)
        
        self.control_sub = self.create_subscription(
            String,
            '/stop_control',
            self.stop_control_cb,
            1)
        
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
        self.mode_service = self.create_client(SetMode, "mavros/set_mode")
        self.done_pub = self.create_publisher(String, "position_reached",qos_profile=1)
        # self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        self.x_val = 0.0
        self.y_val = 0.0
        self.z_val = 1.5
        self.yaw_val = 0
        self.current_state = State()
        self.offb_set_mode = SetMode.Request()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.arm_cmd = CommandBool.Request()
        self.arm_cmd.value = True
            #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = 1.0 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.velocity = Vector3()
        self.yaw = 0.0  #yaw value we send as command
        self.trueYaw = 0.0  #current yaw value of drone
        self.position_status = "Going"



    # def arm_message_callback(self, msg):
    #     self.arm_message = msg.data
    #     self.get_logger().info(f"Arm Message: {self.arm_message}")

    def stop_control_cb(self,msg):
        if(msg.data == "half"):
            self.x_val = self.x_val/2
            self.y_val = self.y_val/2
            self.z_val = 1.5
            self.yaw_val = 0.0
        else:
            self.x_val = 0.0
            self.y_val = 0.0
            self.z_val = 1.5
            self.yaw_val = 0.0
            self.get_logger().info("stopping")
         

    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):
        
        if(self.current_state.mode != "OFFBOARD"):
            pose = PoseStamped()

            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 1.5
            

            for i in range(100):
                self.publisher_trajectory.publish(pose)
                time.sleep(0.01)
                

            reply = self.mode_service.call_async(self.offb_set_mode)
            print(reply)
            self.get_logger().info("Offboard")

        elif(self.current_state.mode == "OFFBOARD" and self.current_state.armed == False):
            self.arm_service.call_async(self.arm_cmd)
            self.get_logger().info("armed")


   

    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):
        self.current_state = msg
        # self.get_logger().info("")


    #receives Twist commands from Teleop and converts NED -> FLU
    def offboard_velocity_callback(self, msg):
        #implements NED -> FLU Transformation
        # self.velocity.x = -msg.linear.y
        # self.velocity.y = msg.linear.x
        # self.velocity.z = -msg.linear.z
        # self.yaw = msg.angular.z
        # self.get_logger().info(str(msg.linear.y))
        # X (FLU) is -Y (NED)
        self.velocity.x = msg.linear.x

        # Y (FLU) is X (NED)
        self.velocity.y = msg.linear.y

        # Z (FLU) is -Z (NED)
        self.velocity.z = msg.linear.z

        # A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
        self.yaw = msg.angular.z

        cos_yaw = np.cos(self.trueYaw)
        sin_yaw = np.sin(self.trueYaw)
        self.velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
        self.velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

        self.x_val += self.velocity_world_x
        self.y_val += self.velocity_world_y
        self.z_val += self.velocity.z
        if(self.z_val < 1.0):
             self.z_val = 1.0
        if(self.z_val > 2.5):
             self.z_val = 2.5

        self.yaw_val += self.yaw

        donemsg = String()
        donemsg.data = "Going"
        self.done_pub.publish(donemsg)
        self.position_status = "Going"

        # self.yaw_val += -self.yaw

    #receives current trajectory values from drone and grabs the yaw value of the orientation
    def attitude_callback(self, msg):
        orientation_q = [0.0,0.0,0.0,0.0]
        orientation_q[0] = msg.pose.orientation.x
        orientation_q[1] = msg.pose.orientation.y
        orientation_q[2] = msg.pose.orientation.z
        orientation_q[3] = msg.pose.orientation.w

        #trueYaw is the drones current yaw value
        pitch, roll, self.trueYaw = euler_from_quaternion(orientation_q)
        if(abs(self.x_val - msg.pose.position.x) < 0.1 and abs(self.y_val - msg.pose.position.y) < 0.1 and self.position_status == "Going"):
             self.get_logger().info("Done")
             donemsg = String()
             donemsg.data = "Done"
             self.done_pub.publish(donemsg)     
             self.position_status = "Done"

        # print(self.trueYaw)
        
        
    #publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if(self.current_state.mode == "OFFBOARD"):
            # Publish offboard control modes

            offboard_msg = PoseStamped()
            offboard_msg.header.frame_id = "odom"
            offboard_msg.header.stamp = self.get_clock().now().to_msg()
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

            self.publisher_trajectory.publish(offboard_msg)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()