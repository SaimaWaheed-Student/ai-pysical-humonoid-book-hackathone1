import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceControllerNode(Node):
    def __init__(self):
        super().__init__('voice_controller_node')
        self.publisher_ = self.create_publisher(String, 'joint_commands', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Voice Controller Node started. Publishing dummy joint commands.')

    def timer_callback(self):
        msg = String()
        # In a real scenario, this would come from a voice recognition system
        # and be processed to generate specific joint commands.
        dummy_command = "move_joint_1_to_0.5_rad"
        msg.data = dummy_command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing dummy command: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
