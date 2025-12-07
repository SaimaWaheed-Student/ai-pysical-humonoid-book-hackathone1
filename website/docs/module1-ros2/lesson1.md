---
sidebar_position: 1
---

# Lesson 1: Introduction to ROS 2

Welcome to the first lesson on ROS 2! This module will introduce you to the fundamental concepts of the Robot Operating System 2.

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a set of software libraries and tools that help you build robot applications. It is a complete rewrite of ROS 1 that is designed to be more robust, secure, and scalable. ROS 2 is built on top of the Data Distribution Service (DDS) standard, which provides a publish-subscribe messaging system for real-time systems.

## Key Concepts

Here are some of the key concepts in ROS 2 that you should be familiar with:

*   **Nodes**: A node is a process that performs some computation. In ROS 2, a robot system is composed of many nodes that communicate with each other.
*   **Topics**: Topics are named buses over which nodes exchange messages. Nodes can publish messages to a topic or subscribe to a topic to receive messages.
*   **Messages**: Messages are the data structures that are sent between nodes over topics. ROS 2 provides a set of standard message types, but you can also define your own custom message types.
*   **Services**: Services are a request-response communication mechanism in ROS 2. One node can offer a service, and another node can make a request to that service and wait for a response.
*   **Actions**: Actions are a long-running, feedback-driven communication mechanism in ROS 2. They are similar to services, but they provide feedback on the progress of the task and can be preempted.

## ROS 2 vs. ROS 1

If you are familiar with ROS 1, you might be wondering what the differences are between ROS 1 and ROS 2. Here are some of the key differences:

*   **DDS**: ROS 2 is built on top of DDS, while ROS 1 has its own custom transport layer.
*   **Python 3**: ROS 2 uses Python 3, while ROS 1 uses Python 2.
*   **Build System**: ROS 2 uses `colcon` as its build system, while ROS 1 uses `catkin`.
*   **Launch System**: ROS 2 has a more flexible and powerful launch system than ROS 1.

## Next Steps

In the next lesson, we will learn how to create a ROS 2 publisher and subscriber.
