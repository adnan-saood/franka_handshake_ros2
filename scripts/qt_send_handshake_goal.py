import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from franka_handshake_msgs.action import Handshake

class HandshakeClient(Node):
    def __init__(self):
        super().__init__('handshake_action_client')
        self._action_client = ActionClient(self, Handshake, 'handshake')

    def send_goal(self, amplitude, frequency, n_oscillations):
        goal_msg = Handshake.Goal()
        goal_msg.amplitude = amplitude
        goal_msg.frequency = frequency
        goal_msg.n_oscillations = n_oscillations

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        print(f"Feedback: progress={feedback_msg.feedback.progress:.2f}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected :(')
            rclpy.shutdown()
            return
        print('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        print(f"Result: success={result.success}, message={result.message}")
        rclpy.shutdown()

def main(args=None):
    import sys
    rclpy.init(args=args)
    node = HandshakeClient()
    node.send_goal(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    rclpy.spin(node)

if __name__ == '__main__':
    main()