# 4.2 Module 1: The Robotic Nervous System (ROS 2 Humble) - Weeks 3-5

ROS 2 (Robot Operating System 2) serves as the communication infrastructure for modern robotics systems, analogous to the nervous system in biological organisms. Students learn to decompose monolithic robotics applications into modular nodes that communicate via standardized message passing.

## Week 3: ROS 2 Architecture and Communication Patterns

**Learning Objectives:**
- Explain the publisher-subscriber pattern and its advantages for robotics
- Create ROS 2 packages with proper dependencies
- Implement talker-listener node pairs using rclpy (Python client library)

**Topics:**
- ROS 2 workspace setup and colcon build system
- Nodes, topics, services, and actions
- Quality of Service (QoS) profiles for reliable vs. best-effort communication
- Visualizing system architecture with rqt_graph

**Lab Exercise**: Students create a temperature sensor simulation node that publishes mock data, and a monitor node that subscribes and logs temperature values. This simple exercise establishes the publish-subscribe mental model used throughout subsequent modules.

**Assessment**: Students submit a ROS 2 package demonstrating bidirectional communication (e.g., command node publishes velocity targets, robot node subscribes and publishes odometry feedback).

## Week 4: Bridging Python AI Agents to ROS 2 Controllers

**Learning Objectives:**
- Integrate Python-based AI/ML code with ROS 2 control systems
- Understand the separation between high-level planning (Python) and low-level control (C++)
- Implement action servers for long-running tasks

**Topics:**
- ROS 2 actions for goal-based behaviors (e.g., "navigate to kitchen")
- Message and service definitions with .msg and .srv files
- Integrating OpenAI API calls within ROS 2 nodes for natural language processing
- Managing execution timing and control rates (Hz)

**Lab Exercise**: Students build a voice-controlled robot simulation where Whisper API transcribes speech, a Python planner node translates commands to motion goals, and a controller node executes movements in Gazebo.

**Industry Relevance**: This architecture mirrors production robotics where perception/planning layers (often Python for ML frameworks) interface with real-time controllers (C++ for determinism). Students learn professional software architecture patterns, not just toy examples.

## Week 5: URDF Robot Modeling and Coordinate Transformations

**Learning Objectives:**
- Define robot kinematic structures using URDF (Unified Robot Description Format)
- Compute forward kinematics from joint angles to end-effector poses
- Use tf2 library for coordinate frame transformations

**Topics:**
- URDF file structure: links, joints, sensors, actuators
- Visual vs. collision geometry
- Coordinate frames: base_link, odom, map, sensor frames
- Broadcasting and listening to tf transformations
- RViz visualization for debugging spatial relationships

**Lab Exercise**: Students model a simplified humanoid robot (torso, arms, head) in URDF, define revolute joints for shoulders and elbows, attach a camera sensor, and visualize in RViz. They then implement a node that computes the camera's position in the world frame given joint angles.

**Common Pitfalls**: Students frequently confuse rotation matrix multiplication order or misunderstand parent-child frame relationships. Visualization in RViz provides immediate feedback to correct misunderstandings.

**Module 1 Deliverable**: Functional ROS 2 package with custom message definitions, publisher-subscriber communication, and URDF robot model validated in RViz. This forms the foundation for all subsequent modules.
