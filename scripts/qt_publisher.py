#!/usr/bin/env python3

import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QLabel,
    QSlider,
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject

# --- ROS2 Node and GUI Integration ---
# To avoid the GUI freezing, the ROS2 node's spin loop runs in a separate thread.
# PyQt signals are used to safely communicate from the ROS thread to the GUI thread if needed,
# though for this simple publisher, we call the ROS node's method directly from the GUI thread.

pub_topic = "/franka_handshake_freq"

freq_range_hz = [0.1, 1.0]  # Min and max frequency in Hz

class RosPublisherNode(Node):
    """
    A simple ROS2 Node that publishes a Float64 value.
    """
    def __init__(self):
        """
        Initializes the ROS2 node and creates a publisher.
        """
        super().__init__('qt_ros_publisher_node')
        # Create a publisher for the pub_topic topic with message type Float64
        self.publisher_ = self.create_publisher(Float64, pub_topic, 10)
        self.get_logger().info('ROS2 Publisher Node has been started.')

    def publish_value(self, value: float):
        """
        Publishes the given float value to the pub_topic topic.
        """
        msg = Float64()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing to: {pub_topic}: {msg.data:.2f}')

class MainWindow(QMainWindow):
    """
    The main GUI window for the application.
    It contains a slider to control the value published by the ROS2 node.
    """
    def __init__(self, ros_node: RosPublisherNode):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("ROS2 Publisher with Qt")
        self.setGeometry(100, 100, 450, 200)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        info_label = QLabel(f"Drag the slider to publish a frequency (Hz) to the {pub_topic} topic.")
        info_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(info_label)

        self.value_label = QLabel(f"Frequency: {freq_range_hz[0]:.2f} Hz")
        self.value_label.setAlignment(Qt.AlignCenter)
        font = self.value_label.font()
        font.setPointSize(20)
        font.setBold(True)
        self.value_label.setFont(font)
        layout.addWidget(self.value_label)

        # Slider setup for frequency range
        self.slider = QSlider(Qt.Horizontal)
        # Map slider range to freq_range_hz
        self.slider_min = int(freq_range_hz[0] * 1000)
        self.slider_max = int(freq_range_hz[1] * 1000)
        self.slider.setRange(self.slider_min, self.slider_max)
        self.slider.setValue(self.slider_min)
        self.slider.setTickInterval((self.slider_max - self.slider_min) // 10)
        self.slider.setTickPosition(QSlider.TicksBelow)
        layout.addWidget(self.slider)

        self.slider.valueChanged.connect(self.on_slider_value_changed)

    def on_slider_value_changed(self, value: int):
        # Convert slider value to frequency in Hz
        float_value = value / 1000.0
        self.value_label.setText(f"Frequency: {float_value:.2f} Hz")
        self.ros_node.publish_value(float_value)

    def closeEvent(self, event):
        print("Closing window and shutting down ROS2 node...")
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main(args=None):
    """
    Main function to initialize ROS2, start the GUI, and manage threads.
    """
    # Initialize the ROS2 client library
    rclpy.init(args=args)

    # Create an instance of our ROS2 node
    ros_publisher_node = RosPublisherNode()

    # Run rclpy.spin() in a separate thread to not block the GUI
    # The daemon=True flag ensures the thread will exit when the main program exits.
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_publisher_node,), daemon=True)
    ros_thread.start()

    # --- Initialize and run the Qt Application ---
    app = QApplication(sys.argv)
    # Pass the ROS node instance to the main window
    window = MainWindow(ros_node=ros_publisher_node)
    window.show()

    # Start the Qt event loop. This is a blocking call.
    # The program will exit when the window is closed, and the closeEvent will handle cleanup.
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
