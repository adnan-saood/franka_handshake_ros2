from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLineEdit, QLabel
)

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

    def launch_robot(self):
        # TODO: Add logic to launch robot bringup
        print("Launching robot bringup...")

    def load_controller(self):
        # TODO: Add logic to load handshake controller
        print("Loading handshake controller...")

    def send_action1(self):
        # TODO: Add logic to send action request to server 1
        print("Sending action request to server 1...")

    def send_action2(self):
        # TODO: Add logic to send action request to server 2
        print("Sending action request to server 2...")

    def toggle_rosbag(self):
        # TODO: Add logic to start/stop rosbag with folder name from self.folder_input.text()
        folder_name = self.folder_input.text()
        print(f"Toggling rosbag collection for folder: {folder_name}")
