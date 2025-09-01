import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from franka_handshake_manager.qt_handshake_manager import Ui_MainWindow
import subprocess

from franka_msgs.srv import ErrorRecovery
from rclpy.action import ActionClient
from franka_handshake_msgs.action import SetGains, Handshake
from franka_msgs.srv import SetFullCollisionBehavior, SetJointStiffness

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from example_interfaces.srv import Trigger, String

import rclpy
import time

bag_topics = ["/joint_states", "/franka_state"]  # Replace with your actual topics
bag_folder = "/home/adnan/handshake_data/"


# Example gains (replace with your desired values or get from UI)
k_gains_stiff = [150.0, 150.0, 150.0, 150.0, 10.0, 6.0, 2.0]
d_gains_stiff = [2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 0.5]

k_gains_compliant = [20.0, 20.0, 20.0, 20.0, 10.0, 6.0, 2.0]
d_gains_compliant = [2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 0.5]

hand_open_state = [0.1, 0.2, 0.2, 0.3]
hand_close_state_stiff = [0.9, 0.9, 0.9, 0.9]
hand_close_state_compliant = [0.4, 0.4, 0.4, 0.9]


def feedback_callback(feedback_msg):
    print(f"Feedback: {feedback_msg.feedback.progress}")

class ApplicationWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Connect button signals to callback stubs
        self.ui.pushButton_bagStart.clicked.connect(self.on_bag_start)
        self.ui.pushButton_bagStop.clicked.connect(self.on_bag_stop)
        self.ui.pushButton_loadHIDControl.clicked.connect(self.on_load_hid_control)
        self.ui.pushButton_loadTactile.clicked.connect(self.on_load_tactile)
        self.ui.pushButton_loadHandshakeController.clicked.connect(self.on_load_handshake_controller)
        self.ui.pushButton_ErrorRecovery.clicked.connect(self.on_error_recovery)
        self.ui.pushButton_setArmStiffness.clicked.connect(self.on_set_arm_stiffness)
        self.ui.pushButton_StartHandshake.clicked.connect(self.on_start_handshake)
        for i in range(1, 9):
            radio_btn = getattr(self.ui, f"radioButton_c{i}")
            radio_btn.toggled.connect(self.on_condition_changed)


        rclpy.init(args=None)
        self.node_ = rclpy.create_node("handshake_manager")

        self.gains_client = ActionClient(self.node_, SetGains, '/set_gains')
        self.error_recovery_client = self.node_.create_client(ErrorRecovery, '/franka_control/error_recovery')

        self.set_safety_params()
    # Callback stubs for button actions
    def on_bag_start(self):
        print("Bag recording started")
        bag_name = self.ui.lineEdit_BagName.text()
        bag_name = bag_name + "_recording" + str(int(time.time()))
        print(f"Recording bag: {bag_name}")
        self.ui.label_recCondition.setText(f"Recording bag: {bag_name}")

        # Trigger tactile node recording via service
        client = self.node_.create_client(String, '/tactile_start_recording')
        if not client.wait_for_service(timeout_sec=2.0):
            print("Tactile recording service not available!")
        else:
            request = String.Request()
            request.data = bag_name
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node_, future)
            if future.result() is not None:
                print(f"Service response: {future.result().success}, {future.result().message}")
            else:
                print("Service call failed")

        # add here ros2 command to trigger bag recording

        cmd = [
            "ros2", "bag", "record",
            "-o", bag_name
        ] + bag_topics

        # Start the ros2 bag record process
        self.bag_process = subprocess.Popen(cmd)
        print(f"Started ros2 bag recording with PID: {self.bag_process.pid}")



    def on_bag_stop(self):
        # TODO: Stop ROS2 bag recording
        print("Bag recording stopped")
        self.ui.label_recCondition.setText("Bag recording stopped")
        self.bag_process.terminate()

        client = self.node_.create_client(Trigger, '/tactile_stop_recording')
        if not client.wait_for_service(timeout_sec=2.0):
            print("Tactile stop recording service not available!")
        else:
            request = Trigger.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node_, future)
            if future.result() is not None:
                print(f"Service response: {future.result().success}, {future.result().message}")
            else:
                print("Service call failed")

    def on_load_hid_control(self):
        # TODO: Load HID control node (ROS2)
        print("HID control loaded")

    def on_load_tactile(self):
        # TODO: Load tactile array listener node (ROS2)
        print("Tactile array listener loaded")

    def on_load_handshake_controller(self):
        # TODO: Load handshake controller node (ROS2)
        print("Handshake controller loaded")

    def on_error_recovery(self):
        success = self.call_error_recovery_service()

    def on_set_arm_stiffness(self):
        goal_msg = SetGains.Goal()
        k_gains_scaled = [k * self.ui.spin_armstiffness.value() for k in k_gains_stiff[:4]] + k_gains_stiff[4:]
        goal_msg.k_gains = k_gains_scaled
        goal_msg.d_gains = d_gains_stiff
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
        print("Arm stiffness set")

    def on_start_handshake(self):
        # TODO: Start handshake action (ROS2)
        print("Handshake started")
        amplitude = self.ui.spin_amplitude.value()
        frequency = self.ui.spin_frequency.value()
        num_handshakes = float(self.ui.spin_nhandshakes.value())
        synchrony = self.ui.spin_synchrony.value()

        print(f"Sending action request to server 1 with parameters: amplitude={amplitude}, frequency={frequency}, num_handshakes={num_handshakes}, synchrony={synchrony}")

        handshake_goal = Handshake.Goal()
        handshake_goal.amplitude = amplitude
        handshake_goal.frequency = frequency
        handshake_goal.n_oscillations = num_handshakes
        handshake_goal.synchrony_factor = synchrony

        hand_stiffness_value = hand_close_state_stiff * self.ui.spin_handstiffness.value()
        self.send_hand_hid_goal_once("close", hand_stiffness_value)

        print("Sending action request to server 1...")
        handshake_action_client = ActionClient(self.node_, Handshake, '/handshake')
        handshake_future = handshake_action_client.send_goal_async(handshake_goal, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(self.node_, handshake_future)
        handshake_goal_handle = handshake_future.result()
        if not handshake_goal_handle.accepted:
            print("Handshake action request rejected.")
        else:
            print("Handshake action request accepted.")

        handshake_result_future = handshake_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node_, handshake_result_future)
        handshake_result = handshake_result_future.result().result
        print(f"Result: success={handshake_result.success}, message={handshake_result.message}")
        self.send_hand_hid_goal_once("open", hand_stiffness_value)

    def on_condition_changed(self):
        selected = self.get_selected_condition()
        self.ui.spin_amplitude.setValue(20)
        self.ui.spin_nhandshakes.setValue(5)
        self.ui.spin_frequency.setValue(0.5)
        synchrony_conditions = [0, 0.7]
        armstiffness_conditions = [0.13, 0.7]
        handstiffness_conditions = [0.2, 0.8]

        # Map condition names to their parameter sets using a dictionary for scalability
        condition_params = {
            f"C{i+1}": [
            synchrony_conditions[i // 4],
            armstiffness_conditions[(i % 4) // 2],
            handstiffness_conditions[i % 2]
            ]
            for i in range(8)
        }

        selected_params = condition_params.get(selected)
        if selected_params:
            self.ui.spin_synchrony.setValue(selected_params[0])
            self.ui.spin_armstiffness.setValue(selected_params[1])
            self.ui.spin_handstiffness.setValue(selected_params[2])

    def get_selected_condition(self):
        for i in range(1, 9):
            radio_btn = getattr(self.ui, f"radioButton_c{i}")
            if radio_btn.isChecked():
                return f"C{i}"
        return None


    def call_error_recovery_service(self):
        if not self.error_recovery_client.wait_for_service(timeout_sec=5.0):
            print("Service not available!")
            return False

        request = ErrorRecovery.Request()
        future = self.error_recovery_client.call_async(request)
        rclpy.spin_until_future_complete(self.node_, future)
        if future.result() is not None:
            print(f"Success: {future.result().success}, Error: {future.result().error}")
            return True
        else:
            print("Service call failed")
            return False

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



    def send_hand_hid_goal_once(self, state, force):
        if not self._action_client.server_is_ready() or self.goal_sent:
            return
        self.goal_sent = True
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [f"mekahand_j{i+1}" for i in range(4)]
        num_points = 255
        self.hand_state = state

        # Determine open/close states based on the 'state' argument
        if state == "open":
            start_state = force  # Assume hand might be closed (stiff)
            end_state = hand_open_state
        elif state == "close":
            start_state = hand_open_state
            end_state = force  # Use force as the close_state array
        else:
            # Default to open if unknown state
            start_state = hand_close_state_stiff
            end_state = hand_open_state
            self.hand_state = "open"

        for i in range(num_points):
            p = JointTrajectoryPoint()
            positions = [
            start_state[j] + (end_state[j] - start_state[j]) * (i / (num_points - 1))
            for j in range(4)
            ]
            p.positions = positions
            t = i * 3.0 / (num_points - 1)
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal_msg.trajectory.points.append(p)
        self.hand_state = state
        self.get_logger().info(f'Sending {"open" if state == "open" else "close"} trajectory goal')
        self._action_client.send_goal_async(goal_msg, feedback_callback=self.hand_feedback_callback)

    def hand_feedback_callback(self, feedback_msg):
        # Print only the sent poses (positions) from feedback
        if hasattr(feedback_msg.feedback, 'actual'):
            positions = getattr(feedback_msg.feedback.actual, 'positions', None)
            if positions is not None:
                self.get_logger().info(f'Sent poses: {positions}')



    def trigger_recording_service(self, filename):
        client = self.node_.create_client(Trigger, '/tactile/start_recording')
        if not client.wait_for_service(timeout_sec=2.0):
            print("Recording service not available!")
            return
        request = Trigger.Request()
        # If your service expects a filename, use a custom service and set request.data = filename
        # For std_srvs/Trigger, no data field is needed
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node_, future)
        if future.result() is not None:
            print(f"Service response: {future.result().success}, {future.result().message}")
        else:
            print("Service call failed")


def main():
    app = QApplication(sys.argv)
    window = ApplicationWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
