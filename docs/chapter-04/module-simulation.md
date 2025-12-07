# 4.3 Module 2: The Digital Twin (Gazebo Classic 11 / Unity) - Weeks 6-7

Physics simulation enables unlimited experimentation without hardware costs or safety risks. Students learn to create virtual environments where algorithms can be tested, debugged, and validated before optional physical deployment.

## Week 6: Gazebo Physics Simulation

**Learning Objectives:**
- Understand physics engine fundamentals (gravity, friction, collisions, inertia)
- Spawn robots and objects in Gazebo programmatically
- Configure sensor plugins for LiDAR, RGB-D cameras, and IMUs
- Apply forces and observe dynamic responses

**Topics:**
- SDF (Simulation Description Format) for world files
- Gazebo plugins for sensors and actuators
- Physics parameters: gravity, friction coefficients, contact dynamics
- Performance optimization: reducing simulation complexity for real-time execution

**Lab Exercise**: Students spawn their URDF humanoid robot from Week 5 in a Gazebo world with obstacles. They implement a node that applies impulse forces to test balance and observe physics-accurate responses (robot falls if forces exceed stability limits).

**Sensor Simulation**: Configure a simulated Intel RealSense D435i depth camera, visualize point cloud data in RViz, and understand the difference between ideal (noise-free) and realistic (noisy) sensor models.

**Common Challenge**: Students often struggle with units (meters vs. centimeters, radians vs. degrees) and physical realism (making robots too light or too powerful). Debugging requires comparing simulation behavior to intuitive physical expectations.

## Week 7: High-Fidelity Rendering with Unity Integration

**Learning Objectives:**
- Integrate Unity game engine for photorealistic rendering
- Generate synthetic datasets for computer vision training
- Understand trade-offs between physics accuracy (Gazebo) and visual fidelity (Unity)

**Topics:**
- Unity Robotics Hub for ROS 2 integration
- Articulation bodies for realistic joint simulation
- Procedural environment generation for domain randomization
- Rendering RGB images, depth maps, and semantic segmentation masks

**Lab Exercise**: Students create a Unity scene resembling a home environment (furniture, appliances) and render synthetic camera data that can train vision models for object detection. They compare detection accuracy on Unity-rendered vs. Gazebo-rendered data.

**Industry Application**: Companies like Tesla and NVIDIA use photorealistic simulation to generate millions of training images for autonomous systems without manual labeling. Students learn cutting-edge techniques used at scale in industry.

**Module 2 Deliverable**: Gazebo world file with functioning sensors and Unity scene integrated with ROS 2, demonstrating ability to simulate complex environments for algorithm testing.
