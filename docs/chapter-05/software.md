# 5.3 Software Stack Setup

## Ubuntu 22.04 LTS + ROS 2 Humble Installation

**Prerequisites**: Clean Ubuntu 22.04 LTS installation with internet connectivity

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop (full installation)
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete -y

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# Initialize rosdep
sudo rosdep init
rosdep update

# Add ROS 2 sourcing to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verification**: `ros2 --version` should return "ros2 cli version: Humble Hawksbill"

## Gazebo Classic 11 Installation

```bash
# Install Gazebo Classic (lighter than Gazebo Fortress for beginner courses)
sudo apt install gazebo11 libgazebo11-dev -y

# Install ROS 2 Gazebo bridge
sudo apt install ros-humble-gazebo-ros-pkgs -y
```

**Test**: `gazebo` should launch empty world

## NVIDIA Isaac Sim Setup (Recommended/Premium Tiers Only)

**Requires**: RTX GPU, Ubuntu 22.04

1. Install Omniverse Launcher from https://www.nvidia.com/en-us/omniverse/
2. Via launcher, install Isaac Sim 2023.1.1+
3. Configure Isaac ROS:

```bash
# Install Isaac ROS dependencies
sudo apt-get install ros-humble-isaac-ros-visual-slam ros-humble-isaac-ros-nvblox
```

**Cloud Alternative**: AWS EC2 G4dn instances with NVIDIA AMI pre-configured

---

## 5.4 Safety Protocols

Physical robotics labs present injury risks requiring structured safety procedures. Even simulation-only courses should teach safety awareness for future industry roles.

### Risk Assessment Matrix

| Activity | Hazard Level | Mitigation | PPE Required |
|----------|--------------|------------|--------------|
| Simulation-only work | **LOW** | Ergonomic workstation setup | None (blue light glasses optional) |
| Edge device testing (Jetson + sensors) | **LOW-MEDIUM** | Battery handling training, secure mounting | Closed-toe shoes when soldering |
| Quadruped robot operation (Unitree Go2) | **MEDIUM** | Safety barriers, E-stop testing, supervised operation | Safety glasses, closed-toe shoes |
| Humanoid robot operation (Unitree G1) | **MEDIUM-HIGH** | Two-person rule, workspace boundaries, emergency procedures | Safety glasses (mandatory), closed-toe shoes (mandatory), lab coat (recommended) |

### Pre-Lab Safety Checklist (Physical Robots Only)

Before any robot operation:
- ☐ Workspace clear of obstructions
- ☐ Emergency stop button tested and within reach
- ☐ PPE worn (safety glasses, closed-toe shoes)
- ☐ Buddy present and briefed on experiment
- ☐ Robot battery charged, secured, and inspected for damage

### Emergency Procedures

**Robot Collision/Unintended Movement**:
1. Press emergency stop button (red mushroom button)
2. Power down robot using proper shutdown procedure
3. Assess for injuries, administer first aid if trained
4. Document incident in safety log
5. Do not resume operation until instructor review

**Battery Fire (LiPo batteries)**:
1. Activate fire alarm
2. Use Class D fire extinguisher (do NOT use water on lithium fires)
3. Evacuate lab immediately

**Electrical Shock**:
1. Do not touch person if still in contact with power source
2. Disconnect power at breaker
3. Call emergency services (campus security/EMS)
4. Administer CPR if trained and necessary

### Documentation Requirements

All incidents (injuries, near-misses, equipment damage) must be documented within 24 hours using university safety reporting systems. This data informs ongoing safety improvements and liability protection.

---

## 5.5 Sample Lab Exercises

Detailed lab exercise examples with learning objectives, step-by-step instructions, expected outputs, and assessment criteria.

### Lab Exercise 1: ROS 2 Talker-Listener (Week 3, Module 1)

**Difficulty**: Beginner | **Time**: 2 hours | **Hardware**: Minimal Tier workstation

**Learning Objectives**:
- Create ROS 2 workspace and package
- Implement publisher and subscriber nodes
- Understand topic-based communication

**Instructions**:
1. Create workspace: `mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src`
2. Create package: `ros2 pkg create --build-type ament_python lab1_talker_listener`
3. Implement talker.py (publishes "Hello World N" at 1 Hz)
4. Implement listener.py (subscribes and prints messages)
5. Build: `colcon build`
6. Test: Terminal 1: `ros2 run lab1_talker_listener talker`, Terminal 2: `ros2 run lab1_talker_listener listener`

**Expected Output**: Listener displays messages with increasing counter

**Assessment**: Code compiles (30%), message rate correct (20%), communication works (30%), code quality (20%)

### Lab Exercise 2: Gazebo Robot Navigation (Week 6, Module 2)

**Difficulty**: Intermediate | **Time**: 4 hours | **Hardware**: Minimal Tier workstation

**Learning Objectives**:
- Spawn robot in Gazebo world
- Subscribe to sensor data (LiDAR)
- Publish velocity commands
- Implement simple obstacle avoidance

**Assessment Rubric**: Robot navigates 10m without collision (40%), LiDAR visualization correct (20%), code modularity (20%), documentation (20%)

---

## 5.6 Remote Learning Adaptations

All core learning objectives achievable remotely through cloud-hosted simulation or local installations on student laptops.

**Cloud Lab Access**: AWS Academy provides education credits. Students access EC2 instances via SSH + VNC for GUI applications (Gazebo, RViz).

**Local Installation**: Ubuntu on Windows via WSL2, macOS via Docker Desktop. Performance reduced but sufficient for learning.

**Equity Considerations**: Loan laptops to students without adequate hardware. Record all lectures for asynchronous access.

---

## Key Takeaways

- **Three budget tiers**: $5K (simulation-only), $15K (full curriculum + edge devices), $50K+ (research-grade with robots)
- **Software stack**: Ubuntu 22.04 LTS + ROS 2 Humble + Gazebo + Isaac Sim (Recommended/Premium tiers)
- **Safety protocols**: Risk assessment, PPE requirements, emergency procedures, incident documentation
- **Remote accessibility**: Cloud instances, local installations ensure equitable access
- **Sample exercises**: Structured labs with clear objectives, instructions, and rubrics

---

## Discussion Questions

1. How should budget allocations change if the institution already owns GPU workstations for gaming/graphics courses?
2. What additional safety protocols might be necessary for elementary/middle school robotics programs versus university settings?
3. How can remote students participate meaningfully in physical robot demonstrations without being on campus?

---

## References (Chapter 5)

NVIDIA. (2024). *Jetson Orin modules*. https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/

Open Robotics. (2024). *ROS 2 Humble installation guide*. https://docs.ros.org/en/humble/Installation.html

Unitree Robotics. (2024). *Unitree Go2 and G1 specifications*. https://www.unitree.com/
