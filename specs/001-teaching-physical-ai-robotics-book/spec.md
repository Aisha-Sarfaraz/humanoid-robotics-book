# Feature Specification: Teaching Physical AI & Humanoid Robotics Book

**Feature Branch**: `001-teaching-physical-ai-robotics-book`
**Created**: December 6, 2025
**Status**: Draft
**Input**: User description: "Create an academic book that addresses a critical gap in computer science education: how to teach Physical AI and Humanoid Robotics in resource-constrained academic environments."

## Document Metadata

- **Project Title:** Teaching Physical AI & Humanoid Robotics: A Resource-Aware Pedagogical Framework
- **Document Type:** High-Level Specification
- **Version:** 1.0
- **Target Platform:** Docusaurus + Spec-Kit Plus → GitHub Pages
- **Governed By:** constitution.md (v1.0)

---

## Clarifications

### Session 2025-12-06

- Q: FR-035 states "Each chapter MUST undergo peer review before integration" but doesn't specify who conducts the review or what the approval criteria are. What peer review process should be used? → A: Option B (Self-review with external validation: subject matter expert reviews technical accuracy of 2-3 key chapters)
- Q: The spec mentions a "6-week development schedule" but doesn't specify daily work hours or whether work is concurrent across chapters or strictly sequential. What is the daily work allocation? → A: Option B (Part-time: 4-6 hours daily, 6 weeks, sequential with overlap where next chapter's research starts while finalizing previous chapter)
- Q: The spec states "Implementation Code: Code examples provided in separate GitHub repository" but doesn't specify repository structure or how code examples map to chapters/labs. What repository organization should be used? → A: Option A (Chapter-aligned structure with separate folders per chapter and subfolders for each lab exercise: `/chapter-4/lab-3/`)
- Q: When literature search uncovers peer-reviewed sources with contradictory findings (e.g., conflicting evidence on simulation vs. physical hardware learning outcomes), how should the book address these conflicts? → A: Option B (Present both perspectives, acknowledge the research conflict, note limitation as area requiring further study)
- Q: FR-031 requires "diagrams MUST use SVG format" and images "minimum 300 DPI with proper attribution" but doesn't specify who creates these visuals or what tools should be used. Who creates diagrams and what tools? → A: Option A (Author creates diagrams using vector graphics tools: draw.io, Inkscape, Figma, or similar)

---

## User Scenarios & Testing

### User Story 1 - Instructor Launches First Physical AI Course (Priority: P1)

A Computer Science professor with ML expertise but limited robotics background wants to offer a Physical AI course next semester within their department's $15,000 lab budget.

**Why this priority**: This is the primary target audience and most critical use case. If an instructor can successfully launch a course using this book, the project achieves its core mission.

**Independent Test**: An instructor can read Chapters 1-5, follow the curriculum framework, procurement recommendations, and safety protocols, then successfully teach their first Physical AI course with measurable student learning outcomes.

**Acceptance Scenarios**:

1. **Given** an instructor with CS background but no robotics experience, **When** they follow the book's curriculum framework (Chapter 4), **Then** they can design a complete 12-week course syllabus with defined learning objectives
2. **Given** a department budget of $5,000-$50,000, **When** the instructor reviews hardware recommendations (Chapter 5), **Then** they can select appropriate infrastructure tier and procure necessary equipment
3. **Given** university safety and liability concerns, **When** the instructor implements the safety protocols (Chapter 5.4), **Then** they can establish a compliant lab environment with documented risk assessments
4. **Given** students with CS background but no robotics experience, **When** the instructor uses the simulation-first pedagogy, **Then** 90% of students successfully complete lab exercises without requiring physical hardware access

---

### User Story 2 - Graduate Student Enters Physical AI Research (Priority: P2)

An MS/PhD student in Computer Science wants to identify a research direction in physical AI and understand the field landscape without scattered tutorial-hopping.

**Why this priority**: Graduate students are secondary audience but critical for long-term field growth. They need structured learning paths and research context.

**Independent Test**: A graduate student can read the book and articulate the physical AI research landscape, identify 2-3 potential thesis topics aligned with their interests, and understand career pathways.

**Acceptance Scenarios**:

1. **Given** a student exploring thesis topics, **When** they review Chapter 2 (Applications) and Chapter 6 (Research Frontiers), **Then** they can identify 3+ specific research problems with evidence of real-world impact
2. **Given** a student unfamiliar with robotics foundations, **When** they work through Chapter 3 (Pedagogical Foundations) and self-assessment framework, **Then** they can identify knowledge gaps and create a personalized learning plan
3. **Given** a student interested in industry careers, **When** they review career pathways (Chapter 6.3), **Then** they understand the job market landscape and required competencies

---

### User Story 3 - Industry Trainer Adapts Curriculum for Corporate Upskilling (Priority: P3)

A corporate trainer or bootcamp instructor needs to condense the Physical AI curriculum into a 2-4 week intensive program for engineers transitioning to robotics roles.

**Why this priority**: Industry trainers represent tertiary audience but validate the framework's flexibility and practical value beyond academia.

**Independent Test**: A trainer can adapt the curriculum framework to an accelerated format while maintaining learning objectives and assessment quality.

**Acceptance Scenarios**:

1. **Given** a 2-week training schedule, **When** the trainer reviews modular curriculum design (Chapter 4), **Then** they can identify core modules and condense non-essential content while preserving learning outcomes
2. **Given** engineers with Python/ML experience, **When** the trainer uses industry-relevant skills emphasis (Chapter 4), **Then** participants can complete hands-on projects demonstrating employable competencies
3. **Given** ROI requirements from corporate stakeholders, **When** the trainer references application domains (Chapter 2), **Then** they can demonstrate clear career pathways and industry demand

---

### Edge Cases

- **What happens when hardware prices fluctuate significantly?** Book provides price ranges with volatility notes and links to vendor sites (Section 5.2); focus on principles over specific costs
- **How does the curriculum handle rapid technological change (e.g., new VLA models)?** Chapter 6.4 addresses evolution strategies; curriculum focuses on foundational principles that remain stable while noting tool versions will change
- **What if a student cannot access simulation hardware (GPU constraints)?** Section 5.6 provides cloud-based alternatives and assessment modifications to ensure equitable access
- **How does the book handle non-English speaking institutions?** While content is in English, pedagogical frameworks and curriculum templates are designed to be culturally adaptable; future derivative works may include translations
- **What if peer-reviewed sources become scarce during literature search?** Risk mitigation addresses this: expand to established conferences, include high-quality technical reports from MIT/Stanford/CMU, use industry white papers as supplementary evidence
- **How should contradictory research findings be handled?** When literature uncovers conflicting peer-reviewed sources (e.g., simulation vs. physical hardware learning outcomes), present both perspectives transparently, acknowledge the research conflict, and note the limitation as an area requiring further study; maintains academic integrity and demonstrates critical thinking

---

## Requirements

### Functional Requirements

#### Content Requirements

- **FR-001**: Book MUST contain 5,000-7,000 words of original content (excluding references, code blocks, appendices)
- **FR-002**: Book MUST include minimum 15 peer-reviewed citations with 50%+ from academic journals (IEEE, ACM, Springer)
- **FR-003**: Book MUST identify and provide evidence for 3+ concrete Physical AI applications (healthcare, manufacturing, service robotics minimum)
- **FR-004**: Book MUST include 7 chapters covering: Introduction, Applications, Pedagogy, Curriculum Design, Implementation, Advanced Topics, Conclusion
- **FR-005**: Book MUST provide a complete 12-week curriculum framework with module breakdowns, learning objectives, and assessment strategies
- **FR-006**: Book MUST include hardware recommendations for 3 budget tiers: Minimal ($5K), Recommended ($15K), Premium ($50K+)
- **FR-007**: Book MUST include 5+ detailed sample lab exercises with learning objectives and assessment criteria
- **FR-008**: Book MUST include safety protocols covering risk assessment, emergency procedures, liability considerations, and student training requirements
- **FR-009**: Book MUST follow APA 7th Edition citation format with complete DOIs for journal articles

#### Quality & Academic Integrity Requirements

- **FR-010**: Book MUST achieve 0% plagiarism detection on Turnitin/Copyscape
- **FR-011**: Book MUST achieve Flesch-Kincaid reading grade level 10-12
- **FR-012**: Book MUST pass academic integrity verification (all citations verified, no fabricated sources)
- **FR-013**: Book MUST include accessibility features compliant with WCAG 2.1 AA standards

#### Technical Platform Requirements

- **FR-014**: Book MUST be published using Docusaurus 2.x/3.x framework
- **FR-015**: Book MUST deploy successfully to GitHub Pages with load time < 3 seconds
- **FR-016**: Book MUST be version controlled via Git with daily commits during development
- **FR-017**: Book MUST include PDF export functionality with intact citations
- **FR-018**: Book MUST include README with usage instructions and license file (recommended: CC BY-NC-SA 4.0)

#### Pedagogical Framework Requirements

- **FR-019**: Curriculum MUST be implementable with simulation-only setup (no physical hardware required as baseline)
- **FR-020**: Curriculum MUST fit within standard 12-week quarter or 15-week semester structure
- **FR-021**: Curriculum MUST provide remote learning alternatives (cloud-based simulation, asynchronous recordings, virtual demonstrations)
- **FR-022**: Assessment methods MUST be equitable (not requiring physical hardware access)
- **FR-023**: Book MUST include rubrics for assessing student projects with criteria for code quality, documentation, and functionality

#### Tools & Technologies Documentation

- **FR-024**: Book MUST document Claude Code usage guidelines including verification workflow, appropriate use cases, and prohibited practices
- **FR-025**: Book MUST document Spec-Kit Plus setup requirements (Node.js 18+, Git, npm/yarn)
- **FR-026**: Book MUST include software stack specifications (Ubuntu 22.04 LTS, ROS 2 Humble, NVIDIA Isaac Sim, Docker)

#### Constitution Compliance Requirements

- **FR-027**: Book MUST achieve citation density of minimum 1 citation per 300 words (Constitution.md line 310)
- **FR-028**: Chapter organization MUST follow 10-15% introduction, 70-75% main content, 10-15% conclusion structure (Constitution.md lines 141-143)
- **FR-029**: Subsection hierarchy MUST NOT exceed 3 levels depth (Constitution.md line 145)
- **FR-030**: Direct quotations MUST NOT exceed 10% of total content (Constitution.md line 262)
- **FR-031**: Images MUST be minimum 300 DPI with proper attribution; diagrams MUST use SVG format created by author using vector graphics tools (draw.io, Inkscape, Figma, or similar) (Constitution.md lines 154-156)
- **FR-032**: Tables MUST include labels, captions, and source citations (Constitution.md line 160)

#### Development Process Requirements

- **FR-033**: Development MUST follow 6-phase workflow: Setup (Days 1-2: environment, tools), Foundational Research (Days 3-7: literature search across 5 domains), User Story Implementation (Days 5-28: incremental content delivery by user story priority), Polish & Quality Assurance (Days 28-42: review, plagiarism checks, peer review, accessibility), with daily Git commits throughout
- **FR-034**: Daily Git commits MUST be maintained throughout development period (Constitution.md line 184)
- **FR-035**: Each chapter MUST undergo peer review before integration; review process consists of self-review using automated tools (Turnitin, Grammarly, readability checkers) plus external validation where subject matter experts review technical accuracy of 2-3 key chapters (Chapters 2, 4, 5 prioritized)
- **FR-036**: Pre-submission quality checklist MUST be completed and documented

#### Source Quality Standards

- **FR-037**: Citation distribution MUST include peer-reviewed journals (50%+), conference proceedings (30%+), technical reports (20% max)
- **FR-038**: Sources older than 10 years MUST include justification for inclusion (Constitution.md line 255)
- **FR-039**: All citations MUST be independently verified against primary sources; no fabricated references permitted

### Key Entities

**Chapter**: Represents a major section of the book (7 total)
- Attributes: Number, Title, Word Count Range, Priority Level, Key Topics, Learning Outcomes
- Relationships: Contains Sections; belongs to Part (Foundations, Curriculum Design, Implementation, Pedagogical Strategies)

**Module**: Represents a 2-3 week instructional unit within the curriculum
- Attributes: Number, Title, Week Range, Learning Objectives, Key Topics, Lab Exercises, Assessment Methods
- Relationships: Belongs to Curriculum Framework; contains Lab Exercises

**Lab Exercise**: Represents a hands-on practical activity for students
- Attributes: Number, Title, Difficulty Level, Estimated Time, Learning Objectives, Required Resources, Assessment Rubric
- Relationships: Belongs to Module; may reference Hardware Tier

**Hardware Tier**: Represents a budget-based infrastructure recommendation
- Attributes: Name (Minimal/Recommended/Premium), Budget Range, Equipment List, Capabilities, Trade-offs
- Relationships: Referenced by Lab Exercises and Implementation Guide

**Citation**: Represents a peer-reviewed academic source
- Attributes: Authors, Year, Title, Journal/Conference, DOI, Source Type (journal/conference/report), Quality Indicators
- Relationships: Supports Claims and Pedagogical Recommendations

**Application Domain**: Represents a real-world use case for Physical AI
- Attributes: Name (Healthcare/Manufacturing/Service), Deployment Statistics, Economic Impact Data, Educational ROI Justification
- Relationships: Contains Case Studies; referenced in Success Criteria

---

## Detailed Specifications

### Chapter Specifications

**Chapter 1: Introduction to Physical AI in Education** (800-1,000 words)

**Learning Objectives:**
- Understand the definition and scope of Physical AI / Embodied Intelligence
- Recognize the resource constraints faced by academic institutions
- Identify the simulation-first pedagogical approach and its rationale

**Key Topics:**
- Physical AI definition: AI systems that understand and interact with physical world
- Resource gap in academia: Limited budgets, lack of robotics expertise in CS departments
- Simulation-first approach: Why virtual environments enable equitable access
- Book structure and how to use this framework

**Citation Requirements:**
- Minimum 3 citations from IEEE/ACM journals defining Physical AI and embodied intelligence
- 1-2 citations on educational resource constraints in CS departments

---

**Chapter 2: Applications of Physical AI** (1,000-1,200 words)

**Learning Objectives:**
- Identify 3+ concrete Physical AI application domains with real-world evidence
- Understand economic impact and deployment statistics for each domain
- Articulate ROI justification for teaching Physical AI in academic settings

**Key Topics:**
- **Healthcare Robotics**: Surgical assistants, rehabilitation robots, elder care automation
- **Manufacturing Automation**: Collaborative robots (cobots), quality inspection, warehouse logistics
- **Service Robotics**: Hospitality, retail, delivery robots, human-robot interaction

**Application Domain Requirements:**
- Each domain MUST include: Deployment statistics, economic impact data (market size, growth rates), peer-reviewed case studies
- Minimum 1 citation per domain from IEEE Transactions or ACM journals

**Citation Requirements:**
- Minimum 5 citations total (healthcare: 2, manufacturing: 2, service robotics: 1)
- At least 3 from IEEE/ACM journals published within last 5 years

---

**Chapter 3: Pedagogical Foundations** (800-1,000 words)

**Learning Objectives:**
- Understand learning theory foundations for hands-on robotics education
- Recognize challenges faced by students without prior robotics background
- Apply self-assessment frameworks to identify student knowledge gaps

**Key Topics:**
- Constructivist learning theory in engineering education
- Simulation-based learning effectiveness vs. physical hardware
- Scaffolding strategies for students with CS-only background
- Remote learning and equitable access considerations

**Citation Requirements:**
- Minimum 4 citations from education technology research (ACM SIGCSE, IEEE Transactions on Education)
- 1-2 citations on simulation-based learning outcomes

---

**Chapter 4: Curriculum Design Framework** (1,200-1,500 words)

**Learning Objectives:**
- Design a 13-week Physical AI course structure from provided framework
- Map 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA) to learning outcomes
- Understand weekly topic progression and dependencies

**Module Breakdown:**

**Weeks 1-2: Foundations**
- Introduction to Physical AI concepts
- Development environment setup (Ubuntu 22.04, Docker, Git)
- Python review for robotics (NumPy, control flow, OOP)

**Module 1: The Robotic Nervous System (ROS 2 Humble) - Weeks 3-5**
- **Week 3**: ROS 2 Nodes, Topics, and Services architecture
- **Week 4**: Bridging Python AI agents to ROS 2 controllers using rclpy
- **Week 5**: Understanding URDF (Unified Robot Description Format) for humanoid robot modeling
- **Learning Outcomes**: Students can create ROS 2 packages, implement publishers/subscribers, define robot models in URDF

**Module 2: The Digital Twin (Gazebo Classic 11/Unity) - Weeks 6-7**
- **Week 6**: Physics simulation in Gazebo (gravity, friction, collisions, inertia)
- **Week 7**: High-fidelity rendering in Unity; sensor simulation (LiDAR, depth cameras, IMUs)
- **Learning Outcomes**: Students can spawn robots in Gazebo, simulate sensor data, validate control algorithms in virtual environments

**Module 3: The AI-Robot Brain (NVIDIA Isaac Platform) - Weeks 8-10**
- **Week 8**: NVIDIA Isaac Sim (Omniverse) for photorealistic simulation and synthetic data generation
- **Week 9**: Isaac ROS: Hardware-accelerated Visual SLAM and perception pipelines
- **Week 10**: Nav2 (Navigation Stack 2) for autonomous path planning and obstacle avoidance
- **Learning Outcomes**: Students can set up Isaac Sim environments, implement VSLAM with Isaac ROS, configure Nav2 for bipedal navigation

**Module 4: Vision-Language-Action (VLA) Models - Weeks 11-12**
- **Week 11**: Voice-to-Action pipeline (OpenAI Whisper for speech recognition, natural language command interpretation)
- **Week 12**: Cognitive Planning with LLMs (translating natural language to ROS 2 action sequences)
- **Capstone Project**: Autonomous humanoid robot receiving voice command ("Go to the kitchen and grab the red cup"), planning path with Nav2, navigating environment, identifying object with Isaac ROS perception, executing manipulation

**Week 13: Conversational AI Integration**
- Multimodal interaction (voice + vision + action)
- Error handling and graceful degradation
- Final project presentations

**Citation Requirements:**
- Minimum 4 citations on curriculum design best practices
- 1-2 citations on modular course structure effectiveness

---

**Chapter 5: Implementation Guide** (1,000-1,200 words)

**Learning Objectives:**
- Select appropriate hardware tier based on budget constraints ($5K, $15K, $50K+)
- Understand software stack requirements and installation procedures
- Apply safety protocols for lab environments

**Key Topics:**
- Hardware tier specifications (detailed below)
- Software stack: Ubuntu 22.04 LTS, ROS 2 Humble Hawksbill, Gazebo Classic 11/Fortress, NVIDIA Isaac Sim 2023.1.1+, Docker
- Lab infrastructure architecture (Sim Rig workstations, Edge Brain devices, sensors, actuators)
- Safety protocols (detailed below)

**Citation Requirements:**
- Minimum 2 citations on robotics lab safety standards
- 1-2 citations on GPU requirements for simulation workloads

---

**Chapter 6: Advanced Topics and Research Frontiers** (600-800 words)

**Learning Objectives:**
- Identify current research challenges in Physical AI
- Understand career pathways (industry vs. academia)
- Recognize areas for potential thesis/capstone projects

**Key Topics:**
- Sim-to-real transfer gap and domain randomization
- Multi-robot coordination and swarm intelligence
- Ethical considerations (safety, bias, job displacement)
- Research frontiers: Whole-body control, dexterous manipulation, human-robot collaboration
- Career pathways: Robotics engineer, ML engineer, research scientist roles

**Citation Requirements:**
- Minimum 3 citations from recent conferences (ICRA, IROS, CoRL)
- 1 citation on robotics job market trends

---

**Chapter 7: Conclusion and Future Directions** (400-600 words)

**Learning Objectives:**
- Synthesize key takeaways from the pedagogical framework
- Understand evolution strategies for curriculum as technology advances

**Key Topics:**
- Summary of simulation-first approach benefits
- Addressing limitations and future enhancements
- Call to action for instructors and institutions
- Resources for staying current (conferences, journals, online communities)

**Citation Requirements:**
- Minimum 1 citation on future of robotics education

---

### Module Specifications (Detailed)

**Module 1: ROS 2 Fundamentals (Weeks 3-5)**

**Prerequisites:** Python programming, basic Linux command line

**Topics:**
1. ROS 2 architecture: Nodes, topics, services, actions, parameters
2. rclpy (ROS 2 Python client library) for creating publishers and subscribers
3. URDF file structure for robot modeling (links, joints, sensors, actuators)
4. Coordinate frame transformations with tf2
5. ROS 2 bag files for recording and playback

**Hands-On Activities:**
- Create a simple talker/listener node pair
- Implement a Python agent that publishes velocity commands to a simulated robot
- Define a URDF model for a simple wheeled robot, then adapt for bipedal humanoid

**Assessment:** Students submit ROS 2 package with custom nodes demonstrating pub/sub communication

---

**Module 2: Gazebo & Unity Simulation (Weeks 6-7)**

**Prerequisites:** Module 1 (ROS 2), basic 3D geometry concepts

**Topics:**
1. Gazebo Classic 11 physics engine: SDF (Simulation Description Format), world files
2. Spawning robots and objects programmatically
3. Sensor plugins: LiDAR (SICK LMS1xx), RGB-D cameras (Intel RealSense D435i simulation), IMUs (BNO055)
4. Unity integration with ROS 2 (Unity Robotics Hub)
5. High-fidelity rendering for synthetic data generation

**Hands-On Activities:**
- Spawn a humanoid robot in Gazebo and apply external forces to test stability
- Simulate a RealSense depth camera and visualize point cloud data
- Create a Unity scene with photorealistic textures for computer vision training

**Assessment:** Students create Gazebo simulation with custom environment and sensor configuration

---

**Module 3: NVIDIA Isaac Platform (Weeks 8-10)**

**Prerequisites:** Module 2 (Gazebo/Unity), ROS 2, understanding of SLAM concepts

**Topics:**
1. NVIDIA Isaac Sim (Omniverse): Photorealistic rendering, physics simulation, synthetic data generation
2. Isaac ROS packages: isaac_ros_visual_slam, isaac_ros_image_pipeline, isaac_ros_nvblox (3D reconstruction)
3. Nav2 navigation stack: Costmaps, global planner (NavFn, Smac Planner), local planner (DWB)
4. Behavior trees for mission planning
5. Integration with Jetson Orin edge devices for on-robot deployment

**Hands-On Activities:**
- Set up Isaac Sim environment with humanoid robot model
- Implement Visual SLAM pipeline using Isaac ROS on simulated sensor data
- Configure Nav2 for bipedal navigation in cluttered environment

**Assessment:** Students develop perception pipeline using Isaac ROS with quantitative accuracy metrics

---

**Module 4: Vision-Language-Action (VLA) Models (Weeks 11-12)**

**Prerequisites:** Module 3 (Isaac Platform), familiarity with OpenAI API or similar LLM services

**Topics:**
1. Speech recognition with OpenAI Whisper (real-time voice command processing)
2. Natural language understanding: Parsing commands like "Go to the kitchen and grab the red cup"
3. LLM-based task planning: Translating high-level goals to ROS 2 action sequences
4. Vision-language grounding: Identifying objects from natural language descriptions
5. Closed-loop control: Action execution, perception feedback, replanning

**Hands-On Activities:**
- Implement voice command interface using Whisper
- Use GPT-4 to decompose natural language commands into ROS 2 action primitives
- Integrate vision (Isaac ROS object detection) + language (GPT) + action (Nav2 + manipulation)

**Assessment (Capstone Project):**
Students build autonomous humanoid system that:
1. Receives voice command via Whisper
2. Plans task sequence using LLM
3. Navigates to target location using Nav2
4. Identifies and manipulates target object using Isaac ROS perception
5. Provides verbal feedback on task completion

**Rubric:**
- Voice recognition accuracy: 20%
- Task planning correctness: 20%
- Navigation success rate: 20%
- Object identification accuracy: 20%
- System integration and error handling: 20%

---

### Lab Exercise Specifications

**Lab Exercise 1: ROS 2 Talker-Listener (Module 1, Week 3)**

**Difficulty:** Beginner
**Estimated Time:** 2 hours
**Learning Objectives:**
- Create ROS 2 nodes using rclpy
- Implement publisher and subscriber
- Understand topic-based communication

**Steps:**
1. Create ROS 2 workspace and package
2. Write talker node that publishes String messages at 1 Hz
3. Write listener node that subscribes and prints messages
4. Test communication using `ros2 topic echo`

**Required Resources:** Minimal Tier hardware (CPU-only acceptable)

**Assessment Rubric:**
- Code compiles without errors: 30%
- Talker publishes at correct rate: 20%
- Listener receives and displays messages: 30%
- Code follows ROS 2 conventions: 20%

---

**Lab Exercise 2: URDF Robot Modeling (Module 1, Week 5)**

**Difficulty:** Intermediate
**Estimated Time:** 4 hours
**Learning Objectives:**
- Understand URDF file structure
- Model robot links, joints, and sensor attachments
- Visualize robot in RViz2

**Steps:**
1. Define base_link, torso, head links with visual and collision geometry
2. Define revolute joints for shoulders, elbows, hips, knees
3. Add sensor elements (camera, LiDAR) with appropriate transforms
4. Launch RViz2 with robot_state_publisher to visualize

**Required Resources:** Minimal Tier hardware

**Assessment Rubric:**
- URDF validates without errors: 25%
- Proper link hierarchy and joint definitions: 25%
- Sensors positioned correctly: 25%
- Visual geometry matches collision geometry: 25%

---

**Lab Exercise 3: Gazebo Physics Simulation (Module 2, Week 6)**

**Difficulty:** Intermediate
**Estimated Time:** 4 hours
**Learning Objectives:**
- Spawn robots in Gazebo programmatically
- Apply forces and observe physics interactions
- Simulate sensor data (IMU, LiDAR)

**Steps:**
1. Create Gazebo world file with obstacles
2. Spawn URDF robot model from Lab 2
3. Write ROS 2 node to apply impulse forces
4. Visualize IMU data and LiDAR point cloud

**Required Resources:** Recommended Tier (GPU beneficial for sensor simulation)

**Assessment Rubric:**
- Robot spawns successfully: 25%
- Physics interactions realistic: 25%
- Sensor data published correctly: 25%
- Code documentation and clarity: 25%

---

**Lab Exercise 4: Isaac ROS Visual SLAM (Module 3, Week 9)**

**Difficulty:** Advanced
**Estimated Time:** 6 hours
**Learning Objectives:**
- Set up Isaac ROS packages
- Implement Visual SLAM pipeline
- Evaluate SLAM accuracy quantitatively

**Steps:**
1. Install Isaac ROS packages and dependencies
2. Configure isaac_ros_visual_slam node with camera parameters
3. Run SLAM on pre-recorded RealSense D435i rosbag data
4. Visualize trajectory in RViz2
5. Compute trajectory error using ground truth

**Required Resources:** Recommended Tier (NVIDIA GPU required, RTX 4070 Ti minimum)

**Assessment Rubric:**
- SLAM pipeline runs without errors: 20%
- Trajectory visualization in RViz2: 20%
- Quantitative error metrics computed: 30%
- Analysis of failure cases: 30%

---

**Lab Exercise 5: Voice-Controlled Navigation (Module 4, Capstone)**

**Difficulty:** Advanced
**Estimated Time:** 10-12 hours (2 weeks)
**Learning Objectives:**
- Integrate voice recognition, LLM planning, navigation, and perception
- Implement closed-loop control with error handling
- Demonstrate system-level robotics competency

**Steps:**
1. Set up Whisper API for voice command recognition
2. Implement LLM-based task planner (GPT-4 API)
3. Configure Nav2 for navigation in Isaac Sim environment
4. Integrate Isaac ROS object detection for target identification
5. Implement error handling (navigation failures, object not found)
6. Test full pipeline with 5+ test commands

**Required Resources:** Premium Tier recommended (RTX 4080+ for real-time performance)

**Assessment Rubric (same as Module 4 Capstone):**
- Voice recognition accuracy: 20%
- Task planning correctness: 20%
- Navigation success rate: 20%
- Object identification accuracy: 20%
- System integration and error handling: 20%

---

### Hardware Tier Specifications

**Minimal Tier ($5,000 budget)**

**Target Use Case:** Simulation-only curriculum, no physical hardware, suitable for introductory courses or resource-constrained departments

**Workstation Specifications (Sim Rig):**
- CPU: Intel Core i7-12700 or AMD Ryzen 7 5800X (8+ cores)
- GPU: NVIDIA GTX 1660 Ti or RTX 3060 (6-12GB VRAM, sufficient for Gazebo Classic, limited Isaac Sim support)
- RAM: 32GB DDR4
- Storage: 512GB NVMe SSD
- OS: Ubuntu 22.04 LTS

**Software Stack:**
- ROS 2 Humble Hawksbill
- Gazebo Classic 11 (primary simulation platform)
- Docker for reproducible environments
- Python 3.10, rclpy, NumPy, OpenCV

**Limitations:**
- No NVIDIA Isaac Sim support (insufficient GPU)
- No physical robot hardware
- Cloud-based alternatives required for advanced perception labs (Module 3)

**Equipment List:**
- 5x Workstations @ $1,000 each = $5,000
- No edge devices, no sensors, no robots

**Capabilities:**
- Modules 1-2 fully supported (ROS 2, Gazebo)
- Module 3 requires cloud instances (AWS EC2 G4dn or equivalent)
- Module 4 feasible with API-based LLM services

---

**Recommended Tier ($15,000 budget)**

**Target Use Case:** Full simulation curriculum with high-fidelity Isaac Sim support, optional edge device integration, suitable for most university CS departments

**Workstation Specifications (Sim Rig):**
- CPU: Intel Core i9-13900K or AMD Ryzen 9 7950X (16+ cores)
- GPU: NVIDIA RTX 4070 Ti or RTX 4080 (12-16GB VRAM, full Isaac Sim support)
- RAM: 64GB DDR5
- Storage: 1TB NVMe Gen4 SSD
- OS: Ubuntu 22.04 LTS

**Edge Device Kit (Optional, for on-robot deployment experiments):**
- Compute: NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB)
- Camera: Intel RealSense Depth Camera D435i
- Microphone: ReSpeaker Mic Array v2.0 (for voice commands)
- IMU: Adafruit BNO055 9-DOF sensor
- Power: Portable power bank or battery management system

**Software Stack:**
- ROS 2 Humble Hawksbill
- Gazebo Classic 11 + Gazebo Fortress (modern GPU physics)
- NVIDIA Isaac Sim 2023.1.1+ (Omniverse)
- Isaac ROS packages (Visual SLAM, nvblox, image pipeline)
- Nav2 navigation stack
- Docker + NVIDIA Container Toolkit
- Unity 2022 LTS (optional, for high-fidelity rendering)

**Robot Option (Budget Permitting):**
- Unitree Go2 quadruped ($1,800-$3,000 depending on configuration)
  - Use as "proxy" for humanoid testing (navigation, perception)
  - Lower cost than full humanoid
  - Good for demonstrating Physical AI concepts

**Equipment List:**
- 3x Workstations @ $3,500 each = $10,500
- 2x Edge Device Kits @ $1,500 each = $3,000
- 1x Unitree Go2 quadruped (optional) = $2,500
- Miscellaneous (cables, mounts, adapters) = $500
- **Total: ~$16,500** (adjust quantities to meet $15K budget)

**Capabilities:**
- All 4 modules fully supported (ROS 2, Gazebo, Isaac Sim, VLA)
- Real-time Isaac Sim rendering and SLAM
- Optional edge device deployment for advanced students
- Physical robot testing (if Go2 purchased)

---

**Premium Tier ($50,000+ budget)**

**Target Use Case:** Research-oriented programs, graduate courses, institutions aiming for humanoid robotics research capabilities

**Workstation Specifications (Sim Rig):**
- CPU: Intel Core i9-14900K or AMD Threadripper (24+ cores)
- GPU: NVIDIA RTX 4090 or RTX 6000 Ada (24-48GB VRAM)
- RAM: 128GB DDR5
- Storage: 2TB NVMe Gen4 SSD (RAID 0 for speed)
- OS: Ubuntu 22.04 LTS

**Edge Device Kit (Production-Grade):**
- Compute: NVIDIA Jetson AGX Orin 64GB (industrial-grade AI computing)
- Camera: Intel RealSense Depth Camera D455 (wider FOV, better range than D435i)
- LiDAR: SLAMTEC RPLIDAR A3 or Velodyne VLP-16 (for robust localization)
- Microphone: ReSpeaker Mic Array v2.0
- IMU: VectorNav VN-100 (industrial-grade INS)
- Force/Torque Sensors: ATI Mini40 (for manipulation tasks)
- Power: High-capacity LiPo battery system with BMS

**Robot Options:**
- **Option A**: Unitree G1 Humanoid Robot (~$16,000)
  - 23 DoF, 1.32m height, mobile manipulation
  - Ideal for humanoid-specific curriculum
- **Option B**: Unitree Go2 Pro + Custom Manipulation Arm (~$8,000 total)
  - Go2 Pro for mobile base ($4,000)
  - 6-DoF robot arm (Kinova Gen3 Lite or similar, $4,000)
  - Modular approach for teaching mobile manipulation separately

**Software Stack (Same as Recommended + Research Tools):**
- All Recommended Tier software
- MoveIt 2 (motion planning framework)
- Behavior Trees (BehaviorTree.CPP)
- ROS 2 Control for hardware abstraction
- NVIDIA Omniverse Isaac Gym (RL training)

**Equipment List:**
- 5x Workstations @ $6,000 each = $30,000
- 3x Edge Device Kits @ $3,500 each = $10,500
- 1x Unitree G1 Humanoid = $16,000
- Lab infrastructure (tables, safety barriers, cameras for recording) = $5,000
- **Total: ~$61,500**

**Capabilities:**
- Full humanoid robotics research pipeline
- Sim-to-real transfer experiments
- Multi-robot coordination (if budget allows multiple robots)
- Publication-quality experimental data
- Graduate thesis projects in Physical AI

---

### Safety Protocol Specifications

**Objective:** Ensure safe operation of robotics labs, minimize injury risk, comply with university liability requirements

**Risk Assessment Categories:**

1. **Simulation-Only Labs (Minimal/Recommended Tiers without physical robots):**
   - **Hazards:** Electrical (workstation power), repetitive strain injury (long coding sessions), eye strain
   - **Risk Level:** LOW
   - **Mitigation:** Ergonomic workstations, regular breaks, electrical safety training

2. **Edge Device Labs (Jetson + sensors, no robot):**
   - **Hazards:** Electrical (batteries, power supplies), sharp sensor edges, data privacy (cameras/microphones)
   - **Risk Level:** LOW-MEDIUM
   - **Mitigation:** Battery handling training, secure data storage, sensor mounting protocols

3. **Physical Robot Labs (Unitree Go2, Unitree G1):**
   - **Hazards:** Collision injury, pinch points (joints), electrical (high-current motors), falling robots
   - **Risk Level:** MEDIUM-HIGH
   - **Mitigation:** Safety barriers, emergency stop systems, PPE (safety glasses, closed-toe shoes), supervised operation only

**Required Safety Training (Before Lab Access):**

1. **General Lab Safety (All Students):**
   - Emergency exits and assembly points
   - Fire extinguisher locations and usage
   - First aid kit location
   - Emergency contact procedures

2. **Electrical Safety (All Tiers):**
   - Proper power supply usage
   - Battery charging protocols (LiPo batteries require fireproof bags)
   - Ground fault circuit interrupter (GFCI) awareness

3. **Robot Operation Safety (Premium Tier Only):**
   - Robot workspace boundaries (marked with floor tape or barriers)
   - Emergency stop (E-stop) button location and testing
   - Proper robot startup/shutdown procedures
   - Never enter robot workspace during autonomous operation
   - Two-person rule for physical robot experiments (one operator, one observer)

**Personal Protective Equipment (PPE):**

- **Simulation Labs:** None required (recommended: blue light glasses for extended screen time)
- **Edge Device Labs:** Closed-toe shoes, safety glasses when soldering or handling electronics
- **Physical Robot Labs:** Closed-toe shoes (MANDATORY), safety glasses (MANDATORY), lab coat (recommended)

**Emergency Procedures:**

1. **Electrical Fire:**
   - Activate fire alarm
   - Use Class C fire extinguisher (do NOT use water)
   - Evacuate lab immediately

2. **Robot Collision/Injury:**
   - Press emergency stop button immediately
   - Administer first aid if qualified
   - Call campus security/EMS (provide university emergency number)
   - Document incident in lab safety log

3. **Equipment Malfunction:**
   - Power down equipment using proper shutdown procedure
   - Tag equipment as "OUT OF SERVICE"
   - Notify lab instructor/supervisor
   - Do NOT attempt repairs without authorization

**Lab Access Control:**

- **Simulation Labs:** Accessible during building hours with student ID
- **Physical Robot Labs:** Restricted access, requires:
  - Completion of safety training (documented with signed form)
  - Instructor supervision for first 3 sessions
  - Buddy system (minimum 2 people present) for all robot operations

**Documentation Requirements:**

- **Pre-Lab Safety Checklist (Physical Robot Labs):**
  - [ ] Workspace clear of obstructions
  - [ ] Emergency stop button tested
  - [ ] PPE worn (safety glasses, closed-toe shoes)
  - [ ] Buddy present and briefed on experiment
  - [ ] Robot battery charged and secured

- **Post-Lab Shutdown Checklist:**
  - [ ] Robot powered down and secured
  - [ ] Batteries removed and stored in fireproof container
  - [ ] Workspace cleaned and organized
  - [ ] Any incidents documented in safety log

**Liability Considerations:**

- Students sign liability waiver before physical robot lab access
- University insurance coverage verified for robotics activities
- Instructor maintains current CPR/First Aid certification
- All incidents documented and reported to department safety officer within 24 hours

**References for Safety Standards:**
- OSHA Guidelines for Laboratory Safety
- ANSI/RIA R15.06 (Industrial Robot Safety Standard, adapted for educational settings)
- University-specific Environmental Health & Safety (EHS) policies

---

## Success Criteria

### Measurable Outcomes

**Content Quality**

- **SC-001**: Book achieves word count between 5,000-7,000 words (excluding references, code)
- **SC-002**: Book includes 15+ peer-reviewed citations with 50%+ from IEEE, ACM, Springer journals
- **SC-003**: Book achieves 0% plagiarism detection on Turnitin/Copyscape
- **SC-004**: Book achieves Flesch-Kincaid Grade 10-12 readability
- **SC-005**: Book includes 3+ Physical AI applications with evidence (deployment statistics, economic impact, peer-reviewed case studies)

**Technical Delivery**

- **SC-006**: Docusaurus builds without errors on first deployment attempt
- **SC-007**: GitHub Pages load time measures < 3 seconds on standard broadband connection
- **SC-008**: WCAG 2.1 AA accessibility compliance achieves 95%+ Lighthouse score
- **SC-009**: All code examples execute successfully in clean environment (100% pass rate)
- **SC-010**: PDF export generates with intact citations and proper formatting

**User Impact (Short-term: 6-12 months)**

- **SC-011**: 5+ universities cite or adopt the framework in course materials
- **SC-012**: GitHub Pages receives 1,000+ page views within 6 months
- **SC-013**: Repository receives 10+ GitHub stars/forks
- **SC-014**: 3+ early adopter instructors provide positive testimonials
- **SC-015**: 5+ GitHub issues/discussions indicate active community engagement

**User Impact (Long-term: 1-3 years)**

- **SC-016**: 20+ documented course implementations based on the framework
- **SC-017**: Book receives academic citations in education technology research
- **SC-018**: Author receives invitations to present at teaching conferences (e.g., ACM SIGCSE)
- **SC-019**: Industry partnerships form based on curriculum framework
- **SC-020**: Derivative works or translated versions emerge

**Pedagogical Effectiveness**

- **SC-021**: Instructors can design complete 12-week course syllabus within one week using framework
- **SC-022**: 90% of students successfully complete simulation-based lab exercises without physical hardware
- **SC-023**: Graduate students can identify 2-3 thesis topics after reading Chapters 2 and 6
- **SC-024**: Industry trainers can adapt curriculum to 2-4 week intensive format while maintaining learning objectives

---

## Assumptions

1. **Target Audience Expertise**: Primary readers (university instructors) have CS background with Python and basic ML knowledge but limited robotics experience
2. **Budget Constraints**: Typical CS department lab budgets range from $5,000-$50,000; recommendations accommodate this range
3. **Technology Evolution**: Specific tool versions (ROS 2 Humble, Isaac Sim 2023.1.1) will change; focuses on principles over versions
4. **Hardware Availability**: Simulation-first approach is viable baseline; physical hardware is enhancement not requirement
5. **Internet Access**: Readers have reliable internet for accessing cloud-based resources, online citations, and GitHub Pages deployment
6. **Academic Integrity Tools**: Access to plagiarism detection tools (Turnitin, Copyscape, or Quetext) for quality assurance
7. **Publication Timeline**: 6-week development schedule is feasible with 4-6 hours daily work allocation; chapters developed sequentially with overlap (start next chapter's literature search while finalizing previous chapter)
8. **Open Source Licensing**: CC BY-NC-SA 4.0 license is appropriate for academic educational resources
9. **Global Accessibility**: Content in English is acceptable; pedagogical frameworks designed for cultural adaptability
10. **AI Tool Availability**: Access to Claude Code for content generation assistance with human oversight and verification
11. **Citation Access**: Author has institutional access to paywalled journals (IEEE Xplore, ACM Digital Library) for citation verification
12. **Version Control**: Git/GitHub proficiency for version control and deployment workflows

---

## Out of Scope

The following are explicitly **NOT** included in this book:

1. **Comprehensive Technical Reference**: Not a complete API reference for all robotics algorithms; focuses on pedagogical methodology
2. **Research Survey**: Not a state-of-the-art survey of humanoid robotics papers; cites relevant research to support teaching strategies
3. **Product Comparisons**: Not detailed vendor comparisons of specific robot models; provides budget-tier recommendations only
4. **Implementation Code**: Code examples provided in separate GitHub repository (chapter-aligned structure: `/chapter-4/lab-3/`), not embedded in book chapters
5. **Mechanical Engineering Deep Dive**: Does not cover kinematics/dynamics beyond what's needed for control programming
6. **Extensive Ethics Discussion**: Brief coverage only; not a comprehensive ethics treatise
7. **K-12 Education**: Focus is exclusively university-level (undergraduate/graduate); does not address pre-college contexts
8. **Hardware Procurement Process**: Does not include RFP templates, vendor negotiation strategies, or purchasing workflows
9. **Accreditation Standards**: Does not address ABET or other accreditation requirements for CS programs
10. **Non-Simulation Platforms**: Does not cover physical-only curriculum designs; simulation-first is core methodology

### Prohibited Sources and Practices

The following sources and practices are **EXPLICITLY PROHIBITED** per Constitution.md standards:

1. **Predatory/Pay-to-Publish Journals**: Any journals from Beall's List or lacking peer review (Constitution.md lines 294-296)
2. **Wikipedia as Primary Source**: May be used for exploration only, not as citable source (Constitution.md lines 297-299)
3. **Social Media and Personal Blogs**: Twitter/X, Medium, personal blogs are NOT acceptable citations (Constitution.md lines 300-302)
4. **Fabricated or Unverified Sources**: All citations MUST be independently verified against primary sources
5. **Excessive Quotation**: Direct quotes MUST NOT exceed 10% of total content (Constitution.md line 262)
6. **Plagiarism**: 0% similarity threshold on plagiarism detection tools; all paraphrasing must be properly cited
7. **AI-Generated Content Without Verification**: Claude Code assistance MUST undergo human review and fact-checking
8. **Outdated Technical Specifications**: Sources older than 10 years require explicit justification (Constitution.md line 255)
9. **Non-Academic Hardware Vendor Marketing**: Hardware recommendations based on specifications, not vendor claims
10. **Unsubstantiated Claims**: All pedagogical claims MUST be supported by peer-reviewed evidence or data

---

## Dependencies

### External Dependencies

1. **Docusaurus Framework**: Requires Docusaurus 2.x or 3.x for site generation
2. **Node.js & npm**: Requires Node.js 18+ and npm for package management
3. **Git & GitHub**: Requires Git for version control and GitHub account for Pages deployment
4. **Claude Code**: Requires Claude Pro/Team subscription for AI-assisted content generation
5. **Spec-Kit Plus**: Requires cloning from https://github.com/panaversity/spec-kit-plus/
6. **Academic Database Access**: Requires institutional access to IEEE Xplore, ACM Digital Library, Google Scholar
7. **Plagiarism Detection Tools**: Requires access to Turnitin, Copyscape, or Quetext
8. **Citation Management**: Optional Zotero or Mendeley for reference organization

### Internal Dependencies

1. **Constitution Document**: Governed by .specify/memory/constitution.md (v1.0) for code quality standards
2. **PHR Templates**: Uses .specify/templates/phr-template.prompt.md for prompt history records
3. **Spec-Kit Plus Templates**: Uses templates from Spec-Kit Plus for academic writing structure
4. **Development Timeline**: 6-week schedule assumes sequential completion of Weeks 1-6 milestones

### Milestone Dependencies

- **Week 2 (Chapters 1-2)** depends on **Week 1 (Literature Search)**: Cannot draft Chapter 2 applications without verified sources
- **Week 3 (Chapters 3-4)** depends on **Week 2**: Pedagogical foundations inform curriculum design
- **Week 4 (Chapter 5)** depends on **Week 3**: Implementation guide references curriculum modules
- **Week 5 (Chapters 6-7)** depends on **Week 4**: Advanced topics build on implementation foundation
- **Week 6 (Deployment)** depends on **Week 5**: Cannot deploy until all content reviewed and validated

---

## Risks & Mitigation

### High-Priority Risks

**Risk 1: Insufficient Peer-Reviewed Sources Found**
- **Probability**: Medium
- **Impact**: High (blocks academic credibility)
- **Mitigation**: Start literature search early (Week 1); expand to conference proceedings (ICRA, IROS, SIGCSE); include high-quality technical reports from MIT, Stanford, CMU as supplementary evidence

**Risk 2: Plagiarism Detected**
- **Probability**: Low
- **Impact**: Critical (invalidates entire work)
- **Mitigation**: Write in own words from the start; cite liberally; run daily plagiarism checks on new content; maintain 0% similarity threshold

**Risk 3: Scope Creep (Exceeds 7,000 Words)**
- **Probability**: High
- **Impact**: Medium (delays timeline, dilutes focus)
- **Mitigation**: Strict outline adherence; move detailed specs to appendices; create separate GitHub wiki for extended content

### Medium-Priority Risks

**Risk 4: Technical Details Become Outdated**
- **Probability**: High
- **Impact**: Medium (reduces long-term utility)
- **Mitigation**: Focus on principles over versions; note in introduction that specific tools evolve; include "last updated" dates for technical specifications

**Risk 5: Docusaurus Build Errors**
- **Probability**: Low
- **Impact**: High (blocks deployment)
- **Mitigation**: Test builds incrementally; use version control for rollback; follow Spec-Kit Plus documentation exactly

**Risk 6: Timeline Slippage**
- **Probability**: Medium
- **Impact**: Medium (delays delivery)
- **Mitigation**: Build in buffer week (Week 6 Days 6-7); prioritize core chapters (2, 4, 5) first; abbreviate Chapter 6 if needed

### Low-Priority Risks

**Risk 7: Code Examples Don't Work**
- **Probability**: Medium
- **Impact**: Medium (damages credibility)
- **Mitigation**: Test in clean environment; use Docker for reproducibility; separate code repo allows independent updates

**Risk 8: Hardware Prices Change Significantly**
- **Probability**: Medium
- **Impact**: Low (doesn't invalidate framework)
- **Mitigation**: Provide price ranges; note volatility; link to vendor sites for current pricing

---

## Notes

### Development Approach

This book project uses **AI-assisted development** (Claude Code) while maintaining rigorous academic standards:

1. **Human-in-the-Loop**: All AI-generated content undergoes critical review and verification
2. **Citation Verification**: Every source independently verified against primary materials (no fabrication)
3. **Plagiarism Prevention**: Daily checks on new content; rewrite any high-similarity sections
4. **Transparency**: AI assistance disclosed in acknowledgments section

### Quality Assurance Process

**Pre-Submission Checklist** includes:
- Content Quality: Word count, citations, plagiarism, readability, evidence support
- Academic Integrity: Citation format, source verification, proper attribution
- Technical Accuracy: Claims verified, specs accurate, code tested, URLs working
- Format & Accessibility: Markdown structure, WCAG compliance, consistent terminology

**Validation Workflow**:
1. Draft chapter → Daily plagiarism check → Commit to Git
2. Complete draft → Technical review → Editorial review → QA checklist
3. Final validation → Constitution.md compliance → Deploy to GitHub Pages

### Timeline Flexibility

The 6-week schedule is **optimistic but achievable** with:
- Daily work allocation (4-6 hours/day)
- Buffer built into Week 6 (Days 6-7)
- Contingency: Prioritize Chapters 2, 4, 5 if timeline at risk

### Success Metrics Phasing

Success criteria are measured across **three time horizons**:
1. **Immediate (Week 6)**: Content quality, technical delivery
2. **Short-term (6-12 months)**: Adoption metrics, community engagement
3. **Long-term (1-3 years)**: Field-level impact, academic recognition

### Future Enhancements (Post-Launch)

Not in scope for v1.0 but potential derivative works:
- Translations into non-English languages
- K-12 adaptation (requires significant pedagogical changes)
- Hardware-first curriculum variant (for well-funded institutions)
- Industry bootcamp version (condensed 2-4 week format)
- Companion video lecture series
- Interactive lab simulation demos
