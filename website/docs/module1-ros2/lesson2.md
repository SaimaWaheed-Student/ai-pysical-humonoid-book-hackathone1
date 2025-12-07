---
sidebar_position: 2
---

# Lesson 2: ROS 2 Publisher and Subscriber

In this lesson, we will learn how to create a ROS 2 publisher and subscriber. This is the most common way for nodes to communicate in ROS 2.

## What is a Publisher?

A publisher is a node that sends messages to a topic. Any node that is subscribed to that topic will receive the messages. Publishing is a useful way to broadcast information to any interested nodes in the system.

Here is an example of how to create a simple publisher in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## What is a Subscriber?

A subscriber is a node that receives messages from a topic. When a message is published to a topic, the subscriber's callback function is executed.

Here is an example of how to create a simple subscriber in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    my_subscriber = MySubscriber()
    rclpy.spin(my_subscriber)
    my_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Publisher and Subscriber

To run the publisher and subscriber, you will need to have two separate terminals. In the first terminal, run the publisher:

```bash
ros2 run my_package my_publisher_node
```

In the second terminal, run the subscriber:

```bash
ros2 run my_package my_subscriber_node
```

You should see the publisher sending messages and the subscriber receiving them.

## Next Steps

In the next lesson, we will learn about ROS 2 services and actions.