---
sidebar_position: 3
---

# Lesson 3: Spawning a Robot in Gazebo

In this lesson, we will learn how to spawn a robot model into our Gazebo simulation. A robot model is typically described using a URDF (Unified Robot Description Format) or SDF file.

## URDF (Unified Robot Description Format)

URDF is an XML format for representing a robot model. In URDF, you can define the robot's links, joints, sensors, and visuals. URDF is widely used in ROS.

Here is a very simple example of a URDF file for a single-link robot:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.03" iyy="0.03" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
</robot>
```

For Gazebo to properly simulate the robot, you often need to add Gazebo-specific tags to the URDF, using the `<gazebo>` element. These tags can define things like physics properties, sensor plugins, and actuator plugins.

## Spawning the Robot

To spawn a URDF model into Gazebo, you can use the `spawn_entity.py` script provided by the `gazebo_ros` package. This script takes the URDF file as input and spawns the model in the simulation.

You can create a ROS 2 launch file to automate this process. The launch file would first start Gazebo and then call the `spawn_entity.py` script.

Here is an example of a launch file that spawns a robot from a URDF file:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the URDF file
    urdf_file_name = 'my_robot.urdf'
    urdf = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        urdf_file_name)

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        # Spawn the robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', urdf],
            output='screen'),
    ])
```

## Conclusion of Module 2

Congratulations on completing Module 2! You have learned the basics of robot simulation with Gazebo. You can now create your own worlds and spawn robots into them. In the next module, we will explore NVIDIA Isaac Sim, another powerful robotics simulator.