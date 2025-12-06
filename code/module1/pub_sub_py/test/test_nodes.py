import pytest
import rclpy
from pub_sub_py.publisher_member_function import MinimalPublisher
from pub_sub_py.subscriber_member_function import MinimalSubscriber

@pytest.fixture(autouse=True)
def initialize_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_minimal_publisher_init():
    node = MinimalPublisher()
    assert node.get_name() == 'minimal_publisher'
    node.destroy_node()

def test_minimal_subscriber_init():
    node = MinimalSubscriber()
    assert node.get_name() == 'minimal_subscriber'
    node.destroy_node()

# Further tests would involve mocking ROS 2 communication,
# which is more complex and typically covered in integration tests.
# For unit tests, we primarily check initialization and basic internal logic.
