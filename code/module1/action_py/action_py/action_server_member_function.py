import time

from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class MinimalActionServer(Node):

    def __init__(self):
        super().__init__('minimal_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i-1])

        feedback_msg = Fibonacci.Feedback()
        for i in range(len(sequence)):
            feedback_msg.sequence = sequence[:i+1]
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        return result


def main(args=None):
    rclpy.init(args=args)

    minimal_action_server = MinimalActionServer()

    rclpy.spin(minimal_action_server)

    minimal_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
