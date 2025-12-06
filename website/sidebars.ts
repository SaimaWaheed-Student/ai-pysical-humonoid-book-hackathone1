import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  bookSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {
        type: 'generated-index',
        title: 'Module 1 Overview',
        description: 'Learn the fundamentals of ROS 2 for robotics.',
        slug: '/module1',
      },
      items: [
        'module1-ros2/lesson1',
        'module1-ros2/lesson2',
        'module1-ros2/lesson3',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      link: {
        type: 'generated-index',
        title: 'Module 2 Overview',
        description: 'Explore simulation environments with Gazebo and Unity.',
        slug: '/module2',
      },
      items: [
        'module2-simulation/lesson1',
        'module2-simulation/lesson2',
        'module2-simulation/lesson3',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      link: {
        type: 'generated-index',
        title: 'Module 3 Overview',
        description: 'Dive into advanced AI for robotics with NVIDIA Isaac.',
        slug: '/module3',
      },
      items: [
        'module3-isaac/lesson1',
        'module3-isaac/lesson2',
        'module3-isaac/lesson3',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      link: {
        type: 'generated-index',
        title: 'Module 4 Overview',
        description: 'Integrate vision, language, and action for autonomous humanoids.',
        slug: '/module4',
      },
      items: [
        'module4-vla/lesson1',
        'module4-vla/lesson2',
        'module4-vla/lesson3',
      ],
    },
  ],
};

export default sidebars;
