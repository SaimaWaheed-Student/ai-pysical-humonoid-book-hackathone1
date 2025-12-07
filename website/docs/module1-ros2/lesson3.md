--- 
sidebar_position: 3
---

# Lesson 3: ROS 2 Services and Actions

In this lesson, we will explore two other important communication mechanisms in ROS 2: services and actions.

## Services

Services are a request-response communication mechanism. One node acts as a service server, providing a service, while another node acts as a service client, making a request and waiting for a response. This is useful for tasks that have a clear beginning and end, and where a synchronous communication pattern is appropriate.

### Service Server

Here is an example of a simple service server in Python that adds two integers:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MyServiceServer(Node):
    def __init__(self):
        super().__init__('my_service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    my_service_server = MyServiceServer()
    rclpy.spin(my_service_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client

Here is the corresponding service client:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MyServiceClient(Node):
    def __init__(self):
        super().__init__('my_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = 41
        self.req.b = 1
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    my_service_client = MyServiceClient()
    my_service_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(my_service_client)
        if my_service_client.future.done():
            try:
                response = my_service_client.future.result()
            except Exception as e:
                my_service_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                my_service_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (my_service_client.req.a, my_service_client.req.b, response.sum))
            break

    my_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions

Actions are used for long-running tasks. They are similar to services, but they provide feedback on the progress of the task and can be preempted (cancelled). An action consists of three parts: a goal, feedback, and a result.

An example of an action is a robot navigating to a specific location. The goal would be the target location. The feedback could be the robot\'s current position. The result would be whether the robot successfully reached the location.

Due to their complexity, we will cover the implementation of actions in a future advanced course.

## Conclusion of Module 1

Congratulations on completing Module 1! You have learned the basics of ROS 2, including nodes, topics, services, and actions. You are now ready to move on to the next module, where we will dive into robot simulation.