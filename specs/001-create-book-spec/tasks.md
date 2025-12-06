# Tasks: Physical AI and Humanoid Robotics Book

**Input**: Design documents from `specs/001-create-book-spec/`
**Prerequisites**: plan.md, spec.md

## Phase 1: Setup Tasks

**Purpose**: Initialize the Docusaurus project and the development environment.

- [X] T001 [P] [US3] Install Node.js and create a new Docusaurus project in `website/`.
- [X] T002 [P] [US2] Configure a custom theme with a dark blue and neon green robotics color scheme in `website/src/css/custom.css`.
- [X] T003 [P] [US2] Install and configure Docusaurus plugins: image optimization, PWA, and search in `website/docusaurus.config.js`.
- [X] T004 [P] Set up the GitHub repository with a basic CI/CD workflow for deploying the Docusaurus site.
- [X] T005 [US3] Create the Docker development environment with Ubuntu 22.04 and ROS 2 Humble in `code/docker/Dockerfile`.
- [X] T006 [US3] Install Gazebo and the ROS 2 Gazebo plugins in the Dockerfile.
- [X] T007 [US3] Install NVIDIA container toolkit dependencies for Isaac Sim in the Dockerfile.
- [X] T008 [P] [US1] Create the initial sidebar structure for the 4 modules in `website/sidebars.js`.
- [X] T009 [P] [US1] Create placeholder markdown files for all 12 lessons in `website/docs/`.
- [X] T010 [P] [US2] Add a 3D model viewer component (e.g., `<model-viewer>`) to the Docusaurus project in `website/src/components/`.

## Phase 2: Module 1 - The Robotic Nervous System (ROS 2)

**Goal**: [US1] Write the content and [US3] create the code examples for the ROS 2 module.

- [X] T011 [US1] Write lesson "Introduction to ROS 2" (1000 words) in `website/docs/module1-ros2/lesson1.md`.
- [X] T012 [P] [US1] Create a comparison table of ROS 1 vs. ROS 2 features in the introduction lesson.
- [X] T013 [US3] Create a ROS 2 package for a Python publisher and subscriber in `code/module1/pub_sub_py/`.
- [X] T014 [US3] Implement the rclpy publisher and subscriber nodes.
- [X] T015 [US3] Add unit tests for the pub/sub nodes and a colcon test script.
- [X] T016 [US3] Create a ROS 2 package for a service server and client in `code/module1/service_py/`.
- [X] T017 [US3] Create a ROS 2 package for an action server and client in `code/module1/action_py/`.
- [X] T018 [US3] Design and build a 20-DOF humanoid URDF model in `code/module1/humanoid_description/urdf/humanoid.urdf`.
- [X] T019 [P] [US2] Create an interactive URDF viewer component in `website/src/components/URDFViewer.js` and embed it in a lesson.
- [>] T020 [P] [US2] Record and edit a video tutorial: "Building a Robot in URDF" (10 min).
- [X] T021 [US3] Develop the voice-controlled joint controller capstone project for this module in `code/module1/voice_controller/`.
- [ ] T022 [P] [US1] Create 10 practice exercises with solutions for the ROS 2 module.
- [ ] T023 [P] [US2] Create the 10-question interactive quiz for Module 1 in `website/src/components/QuizModule1.js`.

## Phase 3: Module 2 - The Digital Twin (Gazebo & Unity)

**Goal**: [US1] Write the content and [US3] create the simulation environments for the second module.

- [ ] T024 [US1] Write Gazebo Fortress installation guide inside the Docker setup instructions.
- [ ] T025 [US3] Create a ROS 2 launch file to start an empty Gazebo world in `code/module2/gazebo_sim/launch/empty_world.launch.py`.
- [ ] T026 [US3] Create a launch file to spawn the humanoid URDF model in Gazebo.
- [ ] T027 [US3] Add a simulated LiDAR sensor (Velodyne VLP-16) to the URDF and configure it in Gazebo.
- [ ] T028 [US3] Add a simulated RealSense D435 depth camera to the URDF and configure it in Gazebo.
- [ ] T029 [US3] Add a simulated IMU with configurable noise to the URDF.
- [ ] T030 [P] [US2] Record and edit a video tutorial: "Tuning Physics Parameters in Gazebo" (8 min).
- [ ] T031 [US1] Write the Unity setup guide: "ROS-Unity in 30 Minutes" in a new lesson file.
- [ ] T032 [US3] Configure the Unity ROS-TCP-Connector and demonstrate communication with the ROS 2 network.
- [ ] T033 [US3] Create the capstone project: Test the robot's stability in 3 different Gazebo environments with varying physics.
- [ ] T034 [P] [US1] Create 10 practice exercises with solutions for the simulation module.
- [ ] T035 [P] [US2] Create the 10-question interactive quiz for Module 2 in `website/src/components/QuizModule2.js`.

## Phase 4: Module 3 - The AI-Robot Brain (NVIDIA Isaac)

**Goal**: [US1] Write the content and [US3] create the AI and perception projects for the third module.

- [ ] T036 [US1] Write the Isaac Sim installation guide (2000 words) as part of the Docker setup.
- [ ] T037 [US3] Create a script to load the humanoid model as a USD file in Isaac Sim in `code/module3/isaac_sim_projects/`.
- [ ] T038 [US3] Write a Python script for domain randomization to generate synthetic data in Isaac Sim.
- [ ] T039 [US3] Generate a dataset of 5000 annotated images for object detection.
- [ ] T040 [US3] Write a guide on training a YOLOv8 model on the generated synthetic data.
- [ ] T041 [US3] Create a ROS 2 package to configure and run Isaac ROS nvblox for VSLAM in `code/module3/isaac_ros_slam/`.
- [ ] T042 [US3] Create a ROS 2 package to configure Nav2 for the humanoid's footprint in `code/module3/humanoid_nav2/`.
- [ ] T043 [US3] Implement a custom path planner plugin for Nav2 suitable for bipedal locomotion.
- [ ] T044 [P] [US2] Record and edit a video tutorial: "GPU-Accelerated Perception with Isaac ROS" (12 min).
- [ ] T045 [US3] Create the capstone project: Have the humanoid navigate an environment with obstacles using the Nav2 stack.
- [ ] T046 [P] [US1] Create 10 practice exercises with solutions for the Isaac module.
- [ ] T047 [P] [US2] Create the 10-question interactive quiz for Module 3 in `website/src/components/QuizModule3.js`.

## Phase 5: Module 4 - Vision-Language-Action (VLA)

**Goal**: [US1] Write the content and [US3] build the end-to-end autonomous humanoid project.

- [ ] T048 [US1] Write the VLA introduction: "The Future of Robotics" (2000 words) in `website/docs/module4-vla/lesson1.md`.
- [ ] T049 [US3] Create a ROS 2 node for OpenAI Whisper integration in `code/module4/vla_nodes/`.
- [ ] T050 [US3] Implement wake word detection using Porcupine or a similar library.
- [ ] T051 [US3] Create a ROS 2 node that sends commands to the GPT-4 API for task decomposition.
- [ ] T052 [US3] Implement the logic to translate a high-level command like "Clean the room" into a sequence of ROS 2 actions.
- [ ] T053 [US3] Create a ROS 2 node that uses CLIP for zero-shot object recognition.
- [ ] T054 [US3] Create a ROS 2 node that uses Grounding DINO for object detection based on text prompts.
- [ ] T055 [US3] Integrate the Segment Anything Model (SAM) for image segmentation.
- [ ] T056 [US3] Build the end-to-end VLA pipeline that connects all the nodes from this module for the capstone project.
- [ ] T057 [P] [US2] Record and edit a video demo of the complete autonomous humanoid (5 min).
- [ ] T058 [P] [US1] Create an evaluation rubric for the final capstone project.
- [ ] T059 [P] [US1] Create 10 practice exercises with solutions for the VLA module.
- [ ] T060 [P] [US2] Create the 10-question interactive quiz for Module 4 in `website/src/components/QuizModule4.js`.

## Phase 6: Polish & Launch

**Purpose**: Finalize the book and prepare for launch.

- [ ] T061 [P] [US3] Write the final Docker deployment guide.
- [ ] T062 [P] [US2] Optimize all images in the Docusaurus site to WebP format.
- [ ] T063 [P] Run a WCAG 2.1 accessibility audit on the deployed site.
- [ ] T064 [P] [US1] Create a comprehensive index for the book.
- [ ] T065 [P] [US1] Write a FAQ page with at least 20 common questions.
- [ ] T066 [P] Record and edit a YouTube trailer for the book (3 min).
- [ ] T067 Deploy the final Docusaurus site to a production environment.
- [ ] T068 [P] Submit the project to Hacker News, Reddit (/r/robotics), and ROS Discourse.

## Dependencies & Execution Order
- **Phase 1 (Setup)** must be completed before any module work can begin.
- **Module Phases (2-5)** can be worked on in parallel to some extent, but the content is progressive, so completing them in order is recommended.
- **Phase 6 (Polish & Launch)** can only begin after all module content and code are complete.
- Within each module, content writing ([US1]) can happen in parallel with code development ([US3]) and interactive component creation ([US2]).
