---
sidebar_position: 1
---

# Lesson 1: Introduction to Gazebo

Welcome to Module 2! In this module, we will learn about robot simulation with Gazebo. Simulation is a crucial tool in robotics for testing and developing algorithms without the need for a physical robot.

## What is Gazebo?

Gazebo is a powerful 3D robotics simulator that allows you to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. It provides a robust physics engine, high-quality graphics, and convenient programmatic and graphical interfaces.

## Why use Gazebo?

*   **Testing**: You can test your robot's control algorithms in a safe and controlled environment.
*   **Development**: You can develop and debug your robot's software without access to the physical hardware.
*   **Sensor Simulation**: Gazebo can simulate a wide variety of sensors, including cameras, lidars, and IMUs.
*   **Cost-effective**: Simulation is much cheaper than building and maintaining a physical robot.

## The Gazebo Interface

When you launch Gazebo, you will be presented with a graphical user interface (GUI). The GUI consists of several components:

*   **3D View**: The main part of the GUI is the 3D view, which shows the simulated world.
*   **Scene Graph**: On the left side of the GUI, you will see the scene graph, which shows all the models in the world.
*   **Toolbar**: At the top of the GUI, you will find a toolbar with tools for manipulating the world and the models in it.
*   **Bottom Panel**: At the bottom of the GUI, you will see a panel that shows information about the simulation time and the real-time factor.

## Running Gazebo with ROS 2

Gazebo is tightly integrated with ROS 2. You can use ROS 2 to control the models in Gazebo and to read the sensor data from the simulated sensors. The `gazebo_ros` package provides the necessary plugins and tools for integrating Gazebo with ROS 2.

To launch an empty world in Gazebo with ROS 2 support, you can run the following command:

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

## Next Steps

In the next lesson, we will learn how to build a custom world in Gazebo.