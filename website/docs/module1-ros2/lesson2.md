---
sidebar_position: 2
---

# Module 1: ROS 2 - Lesson 2: Visualizing URDF Models

This is a placeholder for Lesson 2 of Module 1: The Robotic Nervous System (ROS 2).

## Interactive URDF Model Viewer

Here's an interactive 3D viewer for the humanoid URDF model:

import URDFViewer from '@site/src/components/URDFViewer';

<URDFViewer
  urdf="/urdf/humanoid.urdf"
  scale={1}
  position={[0, 0, 0]}
  rotation={[0, 0, 0]}
/>
