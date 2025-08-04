import sys
import math
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QScrollArea
from PyQt5.QtGui import QPainter, QBrush, QPen, QColor, QFont, QPainterPath
from PyQt5.QtCore import Qt, QPointF, QThread, pyqtSignal

# Import the RosThread from the separate file.
# The RosThread class itself will handle ROS2 availability and dummy mode.
try:
    from UI_model_test_IRL import RosThread
except ImportError:
    print("Error: Could not import UI_model_test_IRL.py. Please ensure it's in the same directory.")
    print("Exiting application.")
    sys.exit(1)

class RobotCanvas(QWidget):
    """
    Custom QWidget for drawing the robot's environment, robot, laser scans, and goal.
    It represents a 5x5 meter area, centered on the canvas.
    """
    MAP_SIZE_METERS = 10.0  # The canvas represents a 10x10 meter area
    PIXELS_PER_METER = 120 # 150 pixels per meter, so a 1500x1500 pixel canvas for 10x10m

    # Define a signal that this canvas will emit when a goal is selected
    goal_selected_signal = pyqtSignal(float, float)
    # Signal to inform parent about robot pose for scrolling (no longer needed for display in MainWindow)
    # robot_pose_updated_signal = pyqtSignal(float, float, float)


    def __init__(self, parent=None):
        super().__init__(parent)
        # Set fixed size based on meters and pixels per meter to enable scrolling
        self.setFixedSize(int(self.MAP_SIZE_METERS * self.PIXELS_PER_METER),
                          int(self.MAP_SIZE_METERS * self.PIXELS_PER_METER))
        self.setBackgroundRole(self.palette().Background)
        self.setAutoFillBackground(True)

        # Robot pose variables (in meters and radians)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0  # Yaw in radians (counter-clockwise from X-axis)

        # Laser scan points (list of (x, y) tuples relative to robot's local frame)
        self.laser_points = []
        # Goal position (in meters)
        self.goal_x = None
        self.goal_y = None

        self.setMouseTracking(True) # Enable mouse tracking for potential future features

    def set_robot_pose(self, x, y, yaw):
        """Updates the robot's pose and triggers a repaint."""
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = -yaw
        self.update() # Trigger repaint
        # Emit signal to parent (MainWindow) to handle scrolling and display update
        # Removed: self.robot_pose_updated_signal.emit(self.robot_x, self.robot_y, self.robot_yaw)


    def set_laser_points(self, points):
        """Updates the laser scan points and triggers a repaint."""
        self.laser_points = points
        self.update() # Trigger repaint

    def set_goal(self, x, y):
        """Sets the goal position and triggers a repaint."""
        self.goal_x = x
        self.goal_y = y
        self.update() # Trigger repaint

    def paintEvent(self, event):
        """
        Handles the drawing of the canvas elements: grid, robot, laser points, and goal.
        """
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # --- Dark Mode Background ---
        painter.fillRect(self.rect(), QColor(30, 30, 30)) # Dark grey background

        # Calculate canvas center in pixels. This corresponds to ROS (0,0)
        center_x_px = self.width() / 2
        center_y_px = self.height() / 2

        # --- Draw Grid (lighter for dark mode) ---
        grid_color = QColor(60, 60, 60) # Slightly lighter grey for grid lines
        grid_pen = QPen(grid_color, 1, Qt.DotLine)
        painter.setPen(grid_pen)

        # Draw horizontal lines for each meter from -5m to +5m
        for i in range(int(-self.MAP_SIZE_METERS / 2), int(self.MAP_SIZE_METERS / 2) + 1):
            y_meter = float(i)
            # Convert meter coordinate to pixel coordinate, remembering Y-axis inversion
            y_px = center_y_px - (y_meter * self.PIXELS_PER_METER)
            painter.drawLine(0, int(y_px), self.width(), int(y_px))

        # Draw vertical lines for each meter from -5m to +5m
        for i in range(int(-self.MAP_SIZE_METERS / 2), int(self.MAP_SIZE_METERS / 2) + 1):
            x_meter = float(i)
            # Convert meter coordinate to pixel coordinate
            x_px = center_x_px + (x_meter * self.PIXELS_PER_METER)
            painter.drawLine(int(x_px), 0, int(x_px), self.height())

        # Draw thicker lines for the 0,0 axes
        axis_pen = QPen(QColor(90, 90, 90), 1, Qt.SolidLine) # Even lighter grey for axes
        painter.setPen(axis_pen)
        painter.drawLine(0, int(center_y_px), self.width(), int(center_y_px)) # X-axis (0m horizontal)
        painter.drawLine(int(center_x_px), 0, int(center_x_px), self.height()) # Y-axis (0m vertical)

        # --- Draw Robot (Stylized Tank - more square) ---
        robot_width_meters = 0.3 # Overall width of the tank
        robot_length_meters = 0.3 # Overall length of the tank
        track_width_meters = 0.05 # Width of each track

        robot_width_px = robot_width_meters * self.PIXELS_PER_METER
        robot_length_px = robot_length_meters * self.PIXELS_PER_METER
        track_width_px = track_width_meters * self.PIXELS_PER_METER

        # Convert robot's ROS coordinates (self.robot_x, self.robot_y) to canvas pixels
        robot_center_px_x = center_x_px - (self.robot_y * self.PIXELS_PER_METER)
        robot_center_px_y = center_y_px - (self.robot_x * self.PIXELS_PER_METER) 

        # Save painter state before rotation
        painter.save()
        painter.translate(robot_center_px_x, robot_center_px_y)
        # Apply an initial -90 degree rotation to make the robot's forward (local X) point upward on canvas
        # Then apply the ROS yaw rotation.
        # ROS positive yaw (CCW) is mapped to Qt's CW rotation, so -math.degrees(self.robot_yaw)
        # To make ROS X-forward align with canvas Y-upward (negative Y), we need an additional -90 degree rotation.
        painter.rotate(math.degrees(self.robot_yaw) - 90)

        # Draw main body (black, less rounded rectangle)
        body_rect_width = robot_width_px - (2 * track_width_px)
        body_rect_height = robot_length_px
        painter.setBrush(QBrush(Qt.black)) # Black robot body
        painter.setPen(QPen(QColor(100, 100, 100), 2)) # Dark grey outline
        painter.drawRoundedRect(
            int(-body_rect_width / 2), int(-body_rect_height / 2),
            int(body_rect_width), int(body_rect_height),
            5, 5 # Less rounded corners for a squarer look
        )

        # Draw tracks (dark grey, rounded rectangles)
        track_y_offset = (body_rect_width / 2) + (track_width_px / 2) # Center of track from robot center

        # Left track (now drawn vertically, relative to the rotated robot)
        painter.setBrush(QBrush(QColor(50, 50, 50))) # Darker grey for tracks
        painter.setPen(QPen(QColor(100, 100, 100), 1))
        painter.drawRoundedRect(
            int(-robot_length_px / 2), int(-track_y_offset - track_width_px / 2),
            int(robot_length_px), int(track_width_px),
            5, 5
        )
        # Right track (now drawn vertically, relative to the rotated robot)
        painter.drawRoundedRect(
            int(-robot_length_px / 2), int(track_y_offset - track_width_px / 2),
            int(robot_length_px), int(track_width_px),
            5, 5
        )

        # Draw robot orientation (a bright line or triangle at the front)
        # A small triangle for direction
        painter.setBrush(QBrush(QColor(255, 165, 0))) # Orange for visibility
        painter.setPen(Qt.NoPen)
        path = QPainterPath()
        front_offset = robot_length_px / 2 - 5
        triangle_base = 10
        path.moveTo(front_offset, 0)
        path.lineTo(front_offset - triangle_base, -triangle_base / 2)
        path.lineTo(front_offset - triangle_base, triangle_base / 2)
        path.closeSubpath()
        painter.drawPath(path)

        # Restore painter state
        painter.restore()

        # --- Draw Laser Scan Points ---
        painter.setBrush(QBrush(QColor(255, 100, 100))) # Brighter red points for dark mode
        painter.setPen(Qt.NoPen) # No outline for points
        point_size = 3 # Size of each laser point

        for px_local, py_local in self.laser_points:
            # Transform laser point from robot's local frame to global frame
            # 1. Rotate the local point by the robot's current yaw
            rotated_x = px_local * math.cos(-self.robot_yaw) - py_local * math.sin(-self.robot_yaw)
            rotated_y = px_local * math.sin(-self.robot_yaw) + py_local * math.cos(-self.robot_yaw)

            # 2. Translate the rotated point by the robot's global position
            global_x = self.robot_y + rotated_y
            global_y = self.robot_x + rotated_x

            # 3. Convert the global point to canvas pixels
            point_px_x = center_x_px - (global_x * self.PIXELS_PER_METER)
            point_px_y = center_y_px - (global_y * self.PIXELS_PER_METER) # Invert Y for canvas

            painter.drawEllipse(QPointF(point_px_x, point_px_y), point_size, point_size)

        # --- Draw Goal ---
        if self.goal_x is not None and self.goal_y is not None:
            # Convert goal's ROS coordinates to canvas pixels
            goal_px_x = center_x_px + (self.goal_x * self.PIXELS_PER_METER)
            goal_px_y = center_y_px - (self.goal_y * self.PIXELS_PER_METER) # Invert Y for canvas

            painter.setPen(QPen(QColor(0, 200, 0), 3)) # Brighter green pen for goal
            painter.setBrush(Qt.NoBrush) # No fill for goal shape
            goal_marker_size = 10 # Size of the goal cross/circle

            # Draw a cross
            painter.drawLine(QPointF(goal_px_x - goal_marker_size, goal_px_y), QPointF(goal_px_x + goal_marker_size, goal_px_y))
            painter.drawLine(QPointF(goal_px_x, goal_px_y - goal_marker_size), QPointF(goal_px_x, goal_px_y + goal_marker_size))
            # Draw a circle around the cross
            painter.drawEllipse(QPointF(goal_px_x, goal_px_y), goal_marker_size, goal_marker_size)


        # --- Draw Current Robot Pose Text ---
        painter.setFont(QFont("Arial", 10))
        painter.setPen(QColor(200, 200, 200)) # Lighter text for dark mode
        painter.drawText(int(robot_center_px_x - 100), int(robot_center_px_y - 80), f"X: {self.robot_x:.2f} m")
        painter.drawText(int(robot_center_px_x - 100), int(robot_center_px_y - 60), f"Y: {self.robot_y:.2f} m")
        painter.drawText(int(robot_center_px_x - 100), int(robot_center_px_y - 40), f"Yaw: {math.degrees(-self.robot_yaw):.2f}°")

    def mousePressEvent(self, event):
        """
        Handles mouse clicks on the canvas to set a new robot goal.
        Converts pixel coordinates to ROS meter coordinates and emits a signal.
        """
        if event.button() == Qt.LeftButton:
            # Calculate canvas center in pixels
            center_x_px = self.width() / 2
            center_y_px = self.height() / 2

            # Get clicked pixel coordinates
            click_px_x = event.pos().x()
            click_px_y = event.pos().y()

            # Convert pixel coordinates back to ROS meter coordinates
            goal_x_meters = (click_px_x - center_x_px) / self.PIXELS_PER_METER
            goal_y_meters = (center_y_px - click_px_y) / self.PIXELS_PER_METER # Invert Y for canvas

            ros_goal_x_meters = -(click_px_y - center_y_px) / self.PIXELS_PER_METER
            ros_goal_y_meters = (center_x_px - click_px_x) / self.PIXELS_PER_METER # Invert Y for canvas

            # Check if the clicked goal is within the 10x10 meter bounds (updated from 5x5)
            half_map = self.MAP_SIZE_METERS / 2.0
            if -half_map <= goal_x_meters <= half_map and \
               -half_map <= goal_y_meters <= half_map:
                self.set_goal(goal_x_meters, goal_y_meters)
                print(f"Canvas: Goal selected: ({goal_x_meters:.2f}, {goal_y_meters:.2f})")
                # Emit the canvas's own signal
                self.goal_selected_signal.emit(goal_x_meters, goal_y_meters)
                print("Canvas: Emitted goal_selected_signal.")
            else:
                # Provide feedback if the goal is out of bounds
                # Access parent's status_label directly, assuming parent is MainWindow
                if self.parent() and hasattr(self.parent(), 'status_label'):
                    self.parent().status_label.setText(f"Status: Goal ({goal_x_meters:.2f}, {goal_y_meters:.2f}) is outside {self.MAP_SIZE_METERS}x{self.MAP_SIZE_METERS}m bounds. Please click inside.")
                print(f"Goal ({goal_x_meters:.2f}, {goal_y_meters:.2f}) is outside {self.MAP_SIZE_METERS}x{self.MAP_SIZE_METERS}m bounds.")


class MainWindow(QMainWindow):
    """
    The main application window for the Robot Navigation GUI.
    Manages the canvas, status label, and the ROS2 thread.
    """
    # Signal to send goal coordinates from GUI to the ROS thread
    publish_goal_signal = pyqtSignal(float, float)
    # New signal to tell the ROS thread to cancel the current goal
    cancel_goal_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Navigation GUI")
        # Make the window bigger
        self.setGeometry(100, 100, 1240, 800) # x, y, width, height of the window

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # Create a QScrollArea and set the RobotCanvas as its widget
        self.scroll_area = QScrollArea(self)
        self.scroll_area.setWidgetResizable(True) # Allow the widget inside to resize
        self.canvas = RobotCanvas(self.scroll_area) # Set scroll_area as parent
        self.scroll_area.setWidget(self.canvas)
        self.layout.addWidget(self.scroll_area) # Add scroll area to main layout

        # Connect the canvas's goal_selected_signal to MainWindow's update_goal_display slot
        self.canvas.goal_selected_signal.connect(self.update_goal_display)
        print("MainWindow: Connected canvas.goal_selected_signal to self.update_goal_display.")

        # Removed: Connect the canvas's robot_pose_updated_signal to MainWindow's update_robot_view
        # self.canvas.robot_pose_updated_signal.connect(self.update_robot_view)
        # print("MainWindow: Connected canvas.robot_pose_updated_signal to self.update_robot_view.")


        # Create a horizontal layout for the goal button and coordinates
        self.goal_control_layout = QHBoxLayout()

        self.send_goal_button = QPushButton("Send Goal")
        self.send_goal_button.setEnabled(False) # Disable until a goal is selected
        self.goal_control_layout.addWidget(self.send_goal_button)

        self.goal_coords_label = QLabel("Goal: Not Selected")
        self.goal_control_layout.addWidget(self.goal_coords_label)

        # New Cancel Goal Button
        self.cancel_goal_button = QPushButton("Cancel Goal")
        self.cancel_goal_button.setEnabled(False) # Disable until a goal is sent
        self.goal_control_layout.addWidget(self.cancel_goal_button)

        self.goal_control_layout.addStretch(1) # Push buttons/label to the left

        self.layout.addLayout(self.goal_control_layout)

        # Removed: Labels for displaying robot's current position
        # self.robot_pos_label_x = QLabel("Robot X: 0.00 m")
        # self.robot_pos_label_y = QLabel("Robot Y: 0.00 m")
        # self.robot_pos_label_yaw = QLabel("Robot Yaw: 0.00°")
        # self.layout.addWidget(self.robot_pos_label_x)
        # self.layout.addWidget(self.robot_pos_label_y)
        # self.layout.addWidget(self.robot_pos_label_yaw)


        # Create a status label to display messages to the user
        self.status_label = QLabel("Initializing...")
        self.layout.addWidget(self.status_label)

        # Initialize and connect the ROS thread
        self.ros_thread = RosThread()
        # Connect signals from ROS thread to update the GUI
        # Odometry signal is now connected to update_robot_view, which then calls canvas.set_robot_pose
        self.ros_thread.odometry_signal.connect(self.canvas.set_robot_pose) # This connection remains to update canvas drawing
        self.ros_thread.laser_scan_signal.connect(self.canvas.set_laser_points)
        self.ros_thread.status_signal.connect(self.update_status)

        # Connect GUI signal to ROS thread method for publishing goals
        self.send_goal_button.clicked.connect(self.on_send_goal_button_clicked)
        self.publish_goal_signal.connect(self.ros_thread.publish_goal)
        # Connect cancel button to its handler and then to ROS thread
        self.cancel_goal_button.clicked.connect(self.on_cancel_goal_button_clicked)
        self.cancel_goal_signal.connect(self.ros_thread.cancel_goal)


        # Store the last selected goal
        self._selected_goal_x = None
        self._selected_goal_y = None

        # Start the ROS thread
        self.ros_thread.start()

        self.update_status("GUI initialized. Waiting for ROS data...")

        # Set initial scroll position to the middle
        # This needs to be done after the canvas has its fixed size set and is added to the scroll area
        self.scroll_area.verticalScrollBar().setValue(
            (self.canvas.height() - self.scroll_area.viewport().height()) // 5
        )
        self.scroll_area.horizontalScrollBar().setValue(
            (self.canvas.width() - self.scroll_area.viewport().width()) // 2
        )
        print("MainWindow: Initial scroll position set to middle.")


    def update_status(self, message):
        """Updates the text in the status label."""
        self.status_label.setText(f"Status: {message}")

    def update_goal_display(self, x, y):
        """Updates the goal coordinates label and enables the send button."""
        print(f"MainWindow: update_goal_display called with ({y:.2f}, {-x:.2f})")
        self._selected_goal_x = x
        self._selected_goal_y = y
        self.goal_coords_label.setText(f"Goal: ({y:.2f}, {-x:.2f})")
        self.send_goal_button.setEnabled(True) # Enable button once a goal is selected
        self.cancel_goal_button.setEnabled(False) # Ensure cancel button is disabled until sent
        print("MainWindow: Goal display updated and button enabled.")

    # Removed: update_robot_view method
    # def update_robot_view(self, x, y, yaw):
    #     """Updates robot position labels and scrolls the canvas to center the robot."""
    #     self.robot_pos_label_x.setText(f"Robot X: {x:.2f} m")
    #     self.robot_pos_label_y.setText(f"Robot Y: {y:.2f} m")
    #     self.robot_pos_label_yaw.setText(f"Robot Yaw: {math.degrees(yaw):.2f}°")
    #
    #     # Calculate target scroll position to center the robot
    #     # Convert robot's ROS coordinates to pixel coordinates on the canvas
    #     # ROS (0,0) is center of map. Canvas (0,0) is top-left.
    #     # Canvas X: robot_x + MAP_SIZE_METERS/2 -> pixel * PIXELS_PER_METER
    #     # Canvas Y: MAP_SIZE_METERS/2 - robot_y -> pixel * PIXELS_PER_METER
    #     robot_px_x = (x + self.canvas.MAP_SIZE_METERS / 2) * self.canvas.PIXELS_PER_METER
    #     robot_px_y = (self.canvas.MAP_SIZE_METERS / 2 - y) * self.canvas.PIXELS_PER_METER
    #
    #     # Get the viewport size of the scroll area
    #     viewport_width = self.scroll_area.viewport().width()
    #     viewport_height = self.scroll_area.viewport().height()
    #
    #     # Calculate the scroll bar values to center the robot
    #     target_h_scroll = int(robot_px_x - viewport_width / 2)
    #     target_v_scroll = int(robot_px_y - viewport_height / 2)
    #
    #     # Set scroll bar values, clamping to valid ranges
    #     self.scroll_area.horizontalScrollBar().setValue(target_h_scroll)
    #     self.scroll_area.verticalScrollBar().setValue(target_v_scroll)


    def on_send_goal_button_clicked(self):
        """Handles the 'Send Goal' button click."""
        if self._selected_goal_x is not None and self._selected_goal_y is not None:
            self.publish_goal_signal.emit(self._selected_goal_x, self._selected_goal_y)
            self.send_goal_button.setEnabled(False) # Disable send button after sending
            self.cancel_goal_button.setEnabled(True) # Enable cancel button after sending
            self.goal_coords_label.setText(f"Goal: Sent! ({self._selected_goal_x:.2f}, {self._selected_goal_y:.2f})") # Update label
            self.update_status("Navigating to goal...") # Change status
            self.canvas.set_goal(self._selected_goal_x, self._selected_goal_y) # Ensure goal is drawn on canvas
        else:
            self.update_status("Please click on the canvas to select a goal first.")

    def on_cancel_goal_button_clicked(self):
        """Handles the 'Cancel Goal' button click."""
        print("MainWindow: Cancel Goal button clicked.")
        self.cancel_goal_signal.emit() # Emit signal to ROS thread
        self.send_goal_button.setEnabled(False) # Keep send disabled until new goal selected
        self.cancel_goal_button.setEnabled(False) # Disable cancel button
        self.goal_coords_label.setText("Goal: Not Selected") # Clear goal display
        self.update_status("Goal cancelled.") # Update status
        self.canvas.set_goal(None, None) # Clear goal from canvas
        self._selected_goal_x = None # Clear stored goal
        self._selected_goal_y = None


    def closeEvent(self, event):
        """
        Handles the window close event. Ensures the ROS thread is terminated gracefully.
        """
        if self.ros_thread.isRunning():
            self.ros_thread.status_signal.emit("Stopping ROS thread...")
            # Call the stop method on the ROS thread to gracefully shut down ROS2
            self.ros_thread.stop()
            self.ros_thread.wait() # Wait for the thread to finish execution
        event.accept()


if __name__ == '__main__':
    # Create the QApplication instance
    app = QApplication(sys.argv)

    # Apply a dark stylesheet to the entire application for dark mode
    app.setStyleSheet("""
        QMainWindow {
            background-color: #2b2b2b; /* Dark background for main window */
            color: #f0f0f0; /* Light text color */
        }
        QWidget { /* Applies to all widgets, including canvas and labels */
            background-color: #2b2b2b;
            color: #f0f0f0;
        }
        QPushButton {
            background-color: #4CAF50; /* Green button */
            color: white;
            border-radius: 5px;
            padding: 8px 15px;
            font-size: 14px;
            border: none; /* Remove default border */
        }
        QPushButton:hover {
            background-color: #45a049;
        }
        QPushButton:pressed {
            background-color: #3e8e41;
        }
        QPushButton:disabled {
            background-color: #555;
            color: #bbb;
        }
        QPushButton#cancelButton { /* Specific style for cancel button */
            background-color: #f44336; /* Red */
        }
        QPushButton#cancelButton:hover {
            background-color: #da190b;
        }
        QPushButton#cancelButton:pressed {
            background-color: #b71c1c;
        }
        QLabel {
            color: #f0f0f0; /* Light text for labels */
        }
    """)

    # Create and show the main window
    window = MainWindow()
    window.show()
    # Start the Qt event loop
    sys.exit(app.exec_())
