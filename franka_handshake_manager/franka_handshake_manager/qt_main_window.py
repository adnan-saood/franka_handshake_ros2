from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLineEdit, QLabel
)
import rclpy
from rclpy.action import ActionClient
from franka_handshake_msgs.action import SetGains

# Example gains (replace with your desired values or get from UI)
k_gains = [24.0, 24.0, 24.0, 24.0, 10.0, 6.0, 2.0]
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

        # Buttons
        self.rosbag_btn = QPushButton("Start/Stop Rosbag")
        self.launch_robot_btn = QPushButton("Launch Robot Bringup")
        self.load_controller_btn = QPushButton("Load Handshake Controller")
        self.send_action1_btn = QPushButton("Send Action Request to Server 1")
        self.send_action2_btn = QPushButton("Send Action Request to Server 2")

        # Add buttons to layout
        main_layout.addWidget(self.rosbag_btn)
        main_layout.addWidget(self.launch_robot_btn)
        main_layout.addWidget(self.load_controller_btn)
        main_layout.addWidget(self.send_action1_btn)
        main_layout.addWidget(self.send_action2_btn)

        # Connect buttons to stub methods
        self.launch_robot_btn.clicked.connect(self.launch_robot)
        self.load_controller_btn.clicked.connect(self.load_controller)
        self.send_action1_btn.clicked.connect(self.send_action1)
        self.send_action2_btn.clicked.connect(self.send_action2)
        self.rosbag_btn.clicked.connect(self.toggle_rosbag)

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        rclpy.init(args=None)
        self.node = rclpy.create_node('qt_handshake_manager')


        self.gains_client = ActionClient(self.node, SetGains, '/set_gains')


    def launch_robot(self):
        # TODO: Add logic to launch robot bringup
        print("Launching robot bringup...")

    def load_controller(self):
        # TODO: Add logic to load handshake controller
        print("Loading handshake controller...")

    def send_action1(self):

        print("Sending action request to server 1...")

    def send_action2(self):
        goal_msg = SetGains.Goal()
        goal_msg.k_gains = k_gains
        goal_msg.d_gains = d_gains
        print("Sending action request to server 2...")

        future = self.gains_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Action request rejected.")
        else:
            print("Action request accepted.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        print(f"Result: success={result.success}, message={result.message}")


    def toggle_rosbag(self):
        # TODO: Add logic to start/stop rosbag with folder name from self.folder_input.text()
        folder_name = self.folder_input.text()
        print(f"Toggling rosbag collection for folder: {folder_name}")
