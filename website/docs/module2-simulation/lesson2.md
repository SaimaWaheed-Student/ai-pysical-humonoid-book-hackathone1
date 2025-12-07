---
sidebar_position: 2
---

# Lesson 2: Building a World in Gazebo

In this lesson, you will learn how to create your own custom world in Gazebo. A world file in Gazebo is an XML file with a `.world` extension that describes everything in a simulation, including robots, lights, sensors, and static objects.

## SDF (Simulation Description Format)

Gazebo uses the Simulation Description Format (SDF) to define worlds, models, and other simulation entities. SDF is an XML format that allows you to describe everything from the visual appearance of an object to its physical properties.

Here is a simple example of an SDF file for a world with a ground plane and a sun:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

## Creating a Custom World

You can create a custom world by writing your own SDF file. You can add models to the world, such as boxes, spheres, and cylinders. You can also add lights, sensors, and other plugins.

Let's create a world with a few simple shapes. Create a file named `my_world.world` and add the following content:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <model name="box">
      <pose>0 2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
    <model name="sphere">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Launching Your Custom World

To launch your custom world, you can use the `gazebo.launch.py` launch file from the `gazebo_ros` package and pass the path to your world file as an argument.

```bash
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/your/my_world.world
```

You should see your custom world with a box and a sphere.

## Next Steps

In the next lesson, we will learn how to spawn a robot in Gazebo.
