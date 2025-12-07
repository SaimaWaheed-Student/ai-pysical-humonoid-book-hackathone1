---
sidebar_position: 3
---

# Lesson 3: ROS 2 Integration with Isaac Sim

One of the most powerful features of Isaac Sim is its seamless integration with ROS and ROS 2. This allows you to leverage the vast ecosystem of ROS packages and tools while taking advantage of Isaac Sim's advanced simulation capabilities.

## The ROS Bridge

Isaac Sim's ROS integration is made possible by the "ROS Bridge", which is a set of extensions that enable communication between Isaac Sim and ROS. The bridge can publish data from the simulator (e.g., camera images, lidar scans) to ROS topics and subscribe to ROS topics to control the robot in the simulation.

## Enabling the ROS 2 Bridge

To use ROS 2 with Isaac Sim, you first need to enable the ROS 2 bridge extension. You can do this from the "Extensions" window in Isaac Sim. Search for "ROS 2 Bridge" and enable it.

## Publishing and Subscribing to Topics

Isaac Sim provides a set of "ROS 2 Components" that you can add to your robot's USD representation. These components allow you to easily publish and subscribe to ROS 2 topics.

For example, to publish camera images, you can add a `ROS2CameraHelper` component to a camera prim in your scene. This component will automatically publish the camera images to a ROS 2 topic.

Here's how you might do it with Python scripting:

```python
# Create a camera
camera = rep.create.camera(
    prim_path="/World/MyCamera",
    position=(2, 0, 1),
    look_at=(0, 0, 0)
)

# Add the ROS 2 camera helper
ros_camera_helper = rep.annotators.get("ROS2CameraHelper")
ros_camera_helper.prim_paths = [camera.prim_path]
ros_camera_helper.topic_name = "my_camera/image_raw"
```

Similarly, to control a robot's joints, you can use the `ROS2JointController` component. This component subscribes to a `JointState` or `Float64MultiArray` topic and applies the received commands to the robot's joints.

## Running ROS 2 Nodes

You can run your ROS 2 nodes as you normally would. Isaac Sim will connect to the ROS 2 network, and your nodes will be able to communicate with the simulator through the ROS 2 bridge.

This makes it incredibly easy to use your existing ROS 2-based control and navigation stacks with Isaac Sim.

## Conclusion of Module 3

Congratulations! You've completed Module 3 and have been introduced to the powerful capabilities of NVIDIA Isaac Sim. You've learned about USD, assets, and how to integrate Isaac Sim with ROS 2. In the final module, we will explore the exciting world of Vision-Language-Action models.