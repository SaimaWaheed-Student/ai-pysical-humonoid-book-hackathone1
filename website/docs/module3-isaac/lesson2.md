---
sidebar_position: 2
---

# Lesson 2: Working with USD and Assets in Isaac Sim

A fundamental concept in NVIDIA Omniverse and Isaac Sim is the Universal Scene Description (USD). In this lesson, we will learn what USD is and how to work with assets in Isaac Sim.

## What is USD?

USD (Universal Scene Description) is an open and extensible file format and framework for describing, composing, and collaborating on 3D scenes. It was developed by Pixar Animation Studios. USD is not just a file format; it is a powerful scene graph that allows you to create complex scenes by layering and composing different elements.

Key features of USD include:

*   **Composition**: You can compose scenes from many different files and sources.
*   **Layering**: You can use layers to non-destructively edit and modify scenes.
*   **Collaboration**: USD is designed for collaborative workflows, allowing multiple artists and developers to work on the same scene simultaneously.
*   **Extensibility**: You can extend USD with your own custom schemas and data types.

## Assets in Isaac Sim

In Isaac Sim, everything in the scene is an asset, including robots, environments, and props. These assets are represented as USD files. Isaac Sim comes with a large library of pre-built assets, but you can also import your own assets from other 3D modeling tools.

You can import assets in formats like FBX, OBJ, and GLTF. When you import an asset, it is converted to USD.

## Working with the Stage

The main 3D scene in Isaac Sim is called the "Stage". You can add assets to the stage by dragging and dropping them from the asset browser. You can also add assets programmatically using Python scripting.

Here is an example of how to add a cube to the stage using Python:

```python
from omni.isaac.core.objects import cuboid

# Add a cube to the stage at position (0, 0, 1)
my_cube = cuboid.VisualCuboid(
    prim_path="/World/MyCube",
    position=(0, 0, 1),
    size=0.5,
    color=(0, 0, 1)
)
```

## Manipulating Assets

You can manipulate assets on the stage using the GUI tools or programmatically with Python. You can move, rotate, and scale assets. You can also change their materials and other properties.

For example, to move the cube we created earlier, you can do the following:

```python
# Get the current position of the cube
position, orientation = my_cube.get_world_pose()

# Set the new position
my_cube.set_world_pose(position=(1, 1, 1), orientation=orientation)
```

## Next Steps

In the next lesson, we will learn how to integrate Isaac Sim with ROS 2.
