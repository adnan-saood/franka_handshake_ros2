from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLineEdit, QLabel, QSlider
)
from PyQt5.QtCore import Qt
import rclpy
from rclpy.action import ActionClient
from franka_handshake_msgs.action import SetGains, Handshake
from franka_msgs.srv import SetFullCollisionBehavior, SetJointStiffness

# Example gains (replace with your desired values or get from UI)
k_gains = [100.0, 100.0, 100.0, 100.0, 10.0, 6.0, 2.0]
d_gains = [2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 0.5]



def feedback_callback(feedback_msg):
    print(f"Feedback: {feedback_msg.feedback.progress}")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Franka Handshake Manager")

        # Central widget and layout
        central_widget = QWidget()
        main_layout = QHBoxLayout()

        # Folder name input for rosbag
        folder_layout = QHBoxLayout()
        folder_label = QLabel("Rosbag Folder Name:")
        self.folder_input = QLineEdit()
        folder_layout.addWidget(folder_label)
        folder_layout.addWidget(self.folder_input)
        main_layout.addLayout(folder_layout)

        # Left panel for buttons
        button_panel = QVBoxLayout()

        self.rosbag_btn = QPushButton("Start/Stop Rosbag")
        self.launch_robot_btn = QPushButton("Launch Robot Bringup")
        self.load_controller_btn = QPushButton("Load Handshake Controller")
        self.send_action1_btn = QPushButton("Send Action Request to Server 1")
        self.send_action2_btn = QPushButton("Send Action Request to Server 2")

        button_panel.addWidget(self.rosbag_btn)
        button_panel.addWidget(self.launch_robot_btn)
        button_panel.addWidget(self.load_controller_btn)
        button_panel.addWidget(self.send_action1_btn)
        button_panel.addWidget(self.send_action2_btn)

        # Sliders for Handshake action (only under send_action1_btn)
        slider_panel = QVBoxLayout()
        slider_panel.setContentsMargins(20, 0, 0, 0)  # Indent for clarity

        self.amplitude_slider = QSlider(Qt.Horizontal)
        self.amplitude_slider.setMinimum(1)
        self.amplitude_slider.setMaximum(100)
        self.amplitude_slider.setValue(5)
        self.amplitude_slider.setTickInterval(1)
        self.amplitude_slider.setTickPosition(QSlider.TicksBelow)
        amplitude_label = QLabel("Amplitude (0.01 - 1.0):")
        self.amplitude_value = QLabel("0.05")

        self.frequency_slider = QSlider(Qt.Horizontal)
        self.frequency_slider.setMinimum(1)
        self.frequency_slider.setMaximum(100)
        self.frequency_slider.setValue(20)
        self.frequency_slider.setTickInterval(1)
        self.frequency_slider.setTickPosition(QSlider.TicksBelow)
        frequency_label = QLabel("Frequency (0.01 - 1.0):")
        self.frequency_value = QLabel("0.20")

        self.n_oscillations_slider = QSlider(Qt.Horizontal)
        self.n_oscillations_slider.setMinimum(1)
        self.n_oscillations_slider.setMaximum(10)
        self.n_oscillations_slider.setValue(3)
        self.n_oscillations_slider.setTickInterval(1)
        self.n_oscillations_slider.setTickPosition(QSlider.TicksBelow)
        n_oscillations_label = QLabel("Oscillations (1 - 10):")
        self.n_oscillations_value = QLabel("3")

        self.synchrony_slider = QSlider(Qt.Horizontal)
        self.synchrony_slider.setMinimum(0)
        self.synchrony_slider.setMaximum(100)
        self.synchrony_slider.setValue(0)
        self.synchrony_slider.setTickInterval(10)
        self.synchrony_slider.setTickPosition(QSlider.TicksBelow)
        synchrony_label = QLabel("Synchrony (0.0 - 1.0):")
        self.synchrony_value = QLabel("0.00")

        # Connect sliders to value labels
        self.amplitude_slider.valueChanged.connect(
            lambda v: self.amplitude_value.setText(f"{v/100:.2f}")
        )
        self.frequency_slider.valueChanged.connect(
            lambda v: self.frequency_value.setText(f"{v/100:.2f}")
        )
        self.n_oscillations_slider.valueChanged.connect(
            lambda v: self.n_oscillations_value.setText(str(v))
        )
        self.synchrony_slider.valueChanged.connect(
            lambda v: self.synchrony_value.setText(f"{v/100:.2f}")
        )

        # Add sliders and labels to panel
        slider_panel.addWidget(amplitude_label)
        slider_panel.addWidget(self.amplitude_slider)
        slider_panel.addWidget(self.amplitude_value)
        slider_panel.addWidget(frequency_label)
        slider_panel.addWidget(self.frequency_slider)
        slider_panel.addWidget(self.frequency_value)
        slider_panel.addWidget(n_oscillations_label)
        slider_panel.addWidget(self.n_oscillations_slider)
        slider_panel.addWidget(self.n_oscillations_value)
        slider_panel.addWidget(synchrony_label)
        slider_panel.addWidget(self.synchrony_slider)
        slider_panel.addWidget(self.synchrony_value)

        # Only show sliders under send_action1_btn
        action1_panel = QVBoxLayout()
        action1_panel.addWidget(self.send_action1_btn)
        action1_panel.addLayout(slider_panel)

        # Replace button_panel's send_action1_btn with action1_panel
        button_panel.insertLayout(3, action1_panel)
        button_panel.removeWidget(self.send_action1_btn)

        main_layout.addLayout(button_panel)
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        rclpy.init(args=None)
        self.node_ = rclpy.create_node('qt_handshake_manager')
        self.gains_client = ActionClient(self.node_, SetGains, '/set_gains')
        self.set_safety_params()

        # Connect buttons to stub methods
        self.launch_robot_btn.clicked.connect(self.launch_robot)
        self.load_controller_btn.clicked.connect(self.load_controller)
        self.send_action1_btn.clicked.connect(self.send_action1)
        self.send_action2_btn.clicked.connect(self.send_action2)
        self.rosbag_btn.clicked.connect(self.toggle_rosbag)

    def launch_robot(self):
        # TODO: Add logic to launch robot bringup
        print("Launching robot bringup...")

    def load_controller(self):
        # TODO: Add logic to load handshake controller
        print("Loading handshake controller...")

    def send_action1(self):
        amplitude = self.amplitude_slider.value() / 100.0
        frequency = self.frequency_slider.value() / 100.0
        n_oscillations = float(self.n_oscillations_slider.value())
        synchrony_factor = self.synchrony_slider.value() / 100.0

        print(f"Sending action request to server 1 with parameters: amplitude={amplitude}, frequency={frequency}, n_oscillations={n_oscillations}, synchrony_factor={synchrony_factor}")

        goal_msg = Handshake.Goal()
        goal_msg.amplitude = amplitude
        goal_msg.frequency = frequency
        goal_msg.n_oscillations = float(n_oscillations)
        goal_msg.synchrony_factor = synchrony_factor

        print("Sending action request to server 1...")
        handshake_client = ActionClient(self.node_, Handshake, '/handshake')
        future = handshake_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(self.node_, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Handshake action request rejected.")
        else:
            print("Handshake action request accepted.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node_, result_future)
        result = result_future.result().result
        print(f"Result: success={result.success}, message={result.message}")

    def send_action2(self):
        goal_msg = SetGains.Goal()
        goal_msg.k_gains = k_gains
        goal_msg.d_gains = d_gains
        print("Sending action request to server 2...")

        future = self.gains_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(self.node_, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Action request rejected.")
        else:
            print("Action request accepted.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node_, result_future)
        result = result_future.result().result
        print(f"Result: success={result.success}, message={result.message}")


    def toggle_rosbag(self):
        # TODO: Add logic to start/stop rosbag with folder name from self.folder_input.text()
        folder_name = self.folder_input.text()
        print(f"Toggling rosbag collection for folder: {folder_name}")




    def set_safety_params(self):
        # Prepare service clients
        collision_client = self.node_.create_client(SetFullCollisionBehavior, '/param_service_server/set_full_collision_behavior')
        stiffness_client = self.node_.create_client(SetJointStiffness, '/param_service_server/set_joint_stiffness')

        # Wait for services
        if not collision_client.wait_for_service(timeout_sec=5.0):
            print("SetFullCollisionBehavior service not available!")
            return
        if not stiffness_client.wait_for_service(timeout_sec=5.0):
            print("SetJointStiffness service not available!")
            return

        # Prepare requests
        collision_req = SetFullCollisionBehavior.Request()
        collision_req.lower_torque_thresholds_acceleration = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        collision_req.upper_torque_thresholds_acceleration = [80.0, 80.0, 80.0, 80.0, 16.0, 14.0, 12.0]
        collision_req.lower_torque_thresholds_nominal = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        collision_req.upper_torque_thresholds_nominal = [80.0, 80.0, 80.0, 80.0, 16.0, 14.0, 12.0]
        collision_req.lower_force_thresholds_acceleration = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        collision_req.upper_force_thresholds_acceleration = [115.0, 275.0, 155.0, 70.0, 25.0, 25.0]
        collision_req.lower_force_thresholds_nominal = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        collision_req.upper_force_thresholds_nominal = [115.0, 275.0, 155.0, 70.0, 25.0, 25.0]

        stiffness_req = SetJointStiffness.Request()
        stiffness_req.joint_stiffness = [3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0]

        # Call services
        collision_future = collision_client.call_async(collision_req)
        stiffness_future = stiffness_client.call_async(stiffness_req)

        rclpy.spin_until_future_complete(self.node_, collision_future)
        if collision_future.result() is not None:
            print("Collision behavior set:", collision_future.result())
        else:
            print("Failed to set collision behavior.")

        rclpy.spin_until_future_complete(self.node_, stiffness_future)
        if stiffness_future.result() is not None:
            print("Joint stiffness set:", stiffness_future.result())
        else:
            print("Failed to set joint stiffness.")
