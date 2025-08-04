import math
import sys
import threading # For managing the ROS2 spinner in a separate thread

# Attempt to import ROS2 components. If not found, a warning will be issued.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rclpy.executors import MultiThreadedExecutor # NEW: For concurrent spinning
    import tf_transformations # ROS2 equivalent for tf.transformations

    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseStamped # NEW: Needed for goal publishing
    from std_msgs.msg import Empty as EmptyMsg # NEW: For cancellation and done status

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 (rclpy, tf_transformations, nav_msgs, sensor_msgs, geometry_msgs, std_msgs) not found.")
    print("Please ensure ROS2 is installed and sourced, and PyQt5 is installed (`pip install PyQt5`).")

from PyQt5.QtCore import QThread, pyqtSignal

class RosThread(QThread):
    """
    A QThread subclass to run ROS2 operations (subscribers, publishers) in a separate thread.
    This prevents the GUI from freezing while waiting for ROS2 messages and allows publishing.
    """
    # Signals to emit data to the GUI thread for updating the canvas
    odometry_signal = pyqtSignal(float, float, float) # x, y, yaw (in radians)
    laser_scan_signal = pyqtSignal(list) # List of (x_local, y_local) points relative to robot
    status_signal = pyqtSignal(str) # For general status messages to the GUI
    navigation_done_signal = pyqtSignal() # NEW: Signal for navigation completion/collision

    def __init__(self):
        super().__init__()
        # Initialize current robot pose and laser points
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0
        self.current_robot_yaw = 0.0 # Yaw in radians
        self.laser_points = [] # Stores (x,y) tuples relative to the robot's local frame
        self._node = None # ROS2 Node instance
        self._executor = None # NEW: ROS2 Executor
        self._running = True # Flag to control thread execution

        # NEW: Publishers and Subscribers
        self._goal_publisher = None
        self._cancel_publisher = None
        self._done_subscriber = None # Renamed for clarity


    def run(self):
        """
        The main execution method for the thread. Initializes ROS2 node,
        subscribers, publishers, and then spins the node using MultiThreadedExecutor.
        """
        if not ROS2_AVAILABLE:
            self.status_signal.emit("ROS2 components not found. Running in dummy mode.")
            # In a real dummy mode, you'd simulate data or indicate non-functionality.
            # For now, it just won't start ROS2.
            return

        try:
            # Initialize ROS2 client library
            if not rclpy.ok():
                rclpy.init(args=None)
            self.status_signal.emit("ROS2 client library initialized.")

            # Create a ROS2 node
            self._node = Node('robot_gui_node')
            self.status_signal.emit("ROS2 node 'robot_gui_node' created.")

            # Define QoS profiles
            # For subscriptions (sensor data like Odometry, LaserScan, and done status)
            qos_profile_sub = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1, # Keep only the latest message
                durability=DurabilityPolicy.VOLATILE
            )
            # For publications (goals, cancellation) - usually more reliable
            qos_profile_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1, # No need for a deep history for simple commands like this
            durability=DurabilityPolicy.TRANSIENT_LOCAL # Keep this volatile to match if you prefer. Or make both transient local.
                                                 # For a simple empty message, VOLATILE is often sufficient IF both are running.
        )

            # Subscribers
            self._node.create_subscription(
                Odometry,
                '/odom', # Common topic for robot odometry
                self.odometry_callback,
                qos_profile_sub
            )
            self._node.create_subscription(
                LaserScan,
                '/front_laser/scan', # Example laser scan topic
                self.laser_scan_callback,
                qos_profile_sub
            )
            # NEW: Subscribe to a topic indicating navigation completion/status
            self._done_subscriber = self._node.create_subscription(
                EmptyMsg,
                '/done_navigating', # This topic should be published by your navigation system
                self.done_navigating_callback,
                qos_profile_sub
            )
            self.status_signal.emit("Subscribed to /odom, /front_laser/scan, and /done_navigating.")

            # NEW: Publishers
            self._goal_publisher = self._node.create_publisher(
                PoseStamped,
                '/goal_pose', # Standard topic for nav2 goals
                qos_profile_pub
            )
            self._cancel_publisher = self._node.create_publisher(
                EmptyMsg,
                '/cancel_navigation', # This topic should be subscribed by your navigation system
                qos_profile_pub
            )
            self.status_signal.emit("Publishers for /goal_pose and /cancel_navigation created.")

            # Use a MultiThreadedExecutor to spin the node
            # This allows callbacks to be processed concurrently and prevents blocking
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)

            self.status_signal.emit("Spinning ROS2 node with MultiThreadedExecutor...")
            self._executor.spin() # This call blocks until shutdown

        except Exception as e:
            self.status_signal.emit(f"Error in ROS2 thread: {e}")
            self._node.get_logger().error(f"Error in ROS2 thread: {e}") # Log error to ROS2 console
        finally:
            self.stop() # Ensure stop routine is called on exit


    def odometry_callback(self, msg):
        """
        Callback function for the /odom topic.
        Extracts robot's position and orientation and emits a signal.
        """
        self.current_robot_x = msg.pose.pose.position.x
        self.current_robot_y = msg.pose.pose.position.y

        # Convert quaternion orientation to Euler angles (roll, pitch, yaw)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # tf_transformations.euler_from_quaternion returns (roll, pitch, yaw)
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)
        self.current_robot_yaw = yaw # Yaw in radians

        # Emit the robot's pose to the GUI thread
        self.odometry_signal.emit(self.current_robot_x, self.current_robot_y, self.current_robot_yaw)

    def laser_scan_callback(self, msg):
        """
        Callback function for the /front_laser/scan topic.
        Processes laser scan data (ranges and angles) and emits a signal.
        """
        points = []
        for i, range_val in enumerate(msg.ranges):
            # Check if the range reading is valid (within min/max limits)
            # and not an 'inf' (infinity) value.
            # Some sensors report 0.0 for invalid, others inf.
            if msg.range_min < range_val < msg.range_max:
                # Calculate the angle for the current scan point
                angle = msg.angle_min + i * msg.angle_increment
                # Convert polar coordinates (range, angle) to Cartesian (x, y)
                # These points are relative to the laser scanner's frame (robot's local frame)
                x_local = range_val * math.cos(angle)
                y_local = range_val * math.sin(angle)
                points.append((x_local, y_local))

        self.laser_points = points
        # Emit the list of local laser points to the GUI thread
        self.laser_scan_signal.emit(self.laser_points)

    def publish_goal(self, x, y):
        """
        Publishes a PoseStamped message to the /goal_pose topic.
        x, y: goal coordinates in meters.
        """
        if not ROS2_AVAILABLE or not self._goal_publisher:
            self.status_signal.emit("ROS2 not available or goal publisher not initialized. Cannot send goal.")
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map' # Assuming goals are given in the 'map' frame
        goal_msg.header.stamp = self._node.get_clock().now().to_msg()
        goal_msg.pose.position.x = y
        goal_msg.pose.position.y = -x
        goal_msg.pose.position.z = 0.0 # 2D navigation

        # For 2D navigation goals, usually the orientation isn't critical unless you need
        # the robot to arrive at a specific heading. Defaulting to no rotation (yaw=0).
        # You could add a yaw parameter if your GUI allows selecting it.
        # tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        goal_msg.pose.orientation.x = quat[0]
        goal_msg.pose.orientation.y = quat[1]
        goal_msg.pose.orientation.z = quat[2]
        goal_msg.pose.orientation.w = quat[3]

        self._goal_publisher.publish(goal_msg)
        self.status_signal.emit(f"Goal published: X={x:.2f}, Y={y:.2f}")
        self._node.get_logger().info(f"Published goal: X={x:.2f}, Y={y:.2f}")

    def cancel_goal(self):
        """
        Publishes an Empty message to a cancellation topic.
        """
        cancel_msg = EmptyMsg()
        self._cancel_publisher.publish(cancel_msg)
        self.status_signal.emit("Cancellation command published.")
        self._node.get_logger().info("Published cancellation command.")

    def done_navigating_callback(self, msg):
        """
        Callback for the '/done_navigating' topic.
        Emits a signal to the GUI when navigation is reported as done.
        """
        self.status_signal.emit("Navigation reported as done.")
        self.navigation_done_signal.emit() # Emit signal to GUI
        self._node.get_logger().info("Received 'done_navigating' signal.")


    def stop(self):
        """
        Stops the ROS2 thread by shutting down the executor, node, and client library.
        """
        self._running = False
        if self._executor:
            # Shutdown the executor first, which will stop the spin
            self._executor.shutdown()
            self.status_signal.emit("ROS2 executor shut down.")
        if self._node:
            self._node.destroy_node() # Destroy the node
            self.status_signal.emit("ROS2 node destroyed.")
        if rclpy.ok():
            rclpy.shutdown() # Shut down the client library
            self.status_signal.emit("ROS2 client library shut down.")
        print("ROS2 thread fully stopped.")
