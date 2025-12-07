# Chapter 5: Implementation Guide

## Overview

This chapter provides practical guidance for establishing Physical AI infrastructure, from minimal budget simulation-only setups to research-grade humanoid robotics labs. It includes hardware specifications, software installation procedures, safety protocols, and sample lab exercises.

**Learning Objectives:**
- Select appropriate infrastructure tier based on budget and educational goals
- Configure software stacks (Ubuntu, ROS 2, Isaac Sim) for robotics development
- Implement safety protocols for lab environments
- Design effective lab exercises with clear learning objectives

**Estimated Reading Time:** 18-22 minutes

---

## 5.1 Lab Infrastructure Options

Universities face varying resource constraints, requiring flexible infrastructure models. The framework presents three tiers accommodating budgets from $5,000 to $50,000+, each delivering educational value appropriate to investment level.

### Infrastructure Philosophy

Rather than viewing budget constraints as limitations, this framework treats them as design parameters. Even the Minimal Tier ($5K) enables complete curriculum delivery through simulation-first approaches. Higher tiers add convenience (faster rendering, parallel experiments) and optional physical validation, but the core learning objectives remain achievable at all levels.

### Cloud vs. On-Premise Trade-offs

**Cloud Infrastructure (AWS, Azure)**:
- **Advantages**: No upfront capital costs, elastic scaling for peak demand (finals week), automatic updates, accessible from anywhere
- **Disadvantages**: Ongoing subscription costs, internet dependency, data privacy concerns for student projects, learning curve for cloud management
- **Best For**: Pilot programs, temporary courses, institutions with robust IT support

**On-Premise Workstations**:
- **Advantages**: One-time purchase, no recurring fees, full control, data stays local, better performance for real-time tasks
- **Disadvantages**: Capital expenditure required, maintenance burden, hardware refresh cycles, limited remote access
- **Best For**: Established programs, institutions with dedicated lab spaces, multi-year commitments

**Hybrid Approach (Recommended)**: On-premise workstations for Modules 1-2 (ROS 2, Gazebo), cloud instances for Module 3 (Isaac Sim) only when local GPUs insufficient. This balances cost and capability.

### Simulation-Only vs. Physical Hardware

**Simulation-Only** (Minimal Tier): Students develop all algorithms in Gazebo/Isaac Sim. Projects demonstrate navigation, perception, and planning in virtual environments. Sufficient for teaching fundamental concepts and industry-standard workflows where simulation precedes hardware deployment.

**Simulation + Edge Devices** (Recommended Tier): Add Jetson Orin development kits and sensors (cameras, IMUs). Advanced students can deploy algorithms to embedded systems, experiencing real-world challenges (sensor calibration, timing constraints) while avoiding costs and safety risks of full robots.

**Simulation + Physical Robots** (Premium Tier): Include quadruped (Unitree Go2, $1,800-$3K) or humanoid (Unitree G1, $16K) robots. Enables complete sim-to-real transfer experiments. Required only for research-focused programs or institutions emphasizing physical demonstration.

---

## 5.2 Hardware Recommendations by Budget

Detailed specifications for three budget tiers, updated as of December 2024. Prices reflect current market rates but include volatility notes given rapid hardware evolution.

### Minimal Tier: $5,000 Budget (Simulation-Only)

**Target Use Case**: Introductory courses, pilot programs, departments testing robotics education before major investment

**Workstation Specifications** (5 student workstations @ $1,000 each):
- **CPU**: Intel Core i7-12700 or AMD Ryzen 7 5800X (8+ cores, 3.6+ GHz base)
- **GPU**: NVIDIA GTX 1660 Ti or RTX 3060 (6-12GB VRAM)
  - Sufficient for Gazebo Classic 11 and basic RViz visualization
  - Insufficient for Isaac Sim (requires RTX series with ray tracing)
- **RAM**: 32GB DDR4 (minimum for running ROS 2 + Gazebo + IDE + browser simultaneously)
- **Storage**: 512GB NVMe SSD (fast builds and simulation load times critical for productivity)
- **OS**: Ubuntu 22.04 LTS (official ROS 2 Humble support)

**Software Stack**:
- ROS 2 Humble Hawksbill (LTS through May 2027)
- Gazebo Classic 11 (mature, well-documented, lighter than Gazebo Fortress)
- Docker + Docker Compose (reproducible environments)
- Python 3.10, Visual Studio Code, Git

**Capabilities**:
- ✅ Modules 1-2 fully supported (ROS 2, Gazebo simulation)
- ⚠️ Module 3 requires cloud instances (AWS EC2 G4dn.xlarge @ $0.526/hour for Isaac Sim)
- ✅ Module 4 feasible using API-based LLMs (OpenAI, Anthropic)

**Cost Breakdown**:
- 5× Workstations: $5,000
- Software: $0 (all open-source)
- Cloud computing (Module 3, ~30 hours/student): $474/student (can share instances or use AWS Educate credits)

**Limitations**: No physical robots, no edge device deployment, cloud dependency for advanced perception labs.

---

### Recommended Tier: $15,000 Budget (Full Curriculum + Optional Edge Devices)

**Target Use Case**: Standard university CS departments, established courses with 10-20 students

**Workstation Specifications** (3 high-performance workstations @ $3,500 each):
- **CPU**: Intel Core i9-13900K or AMD Ryzen 9 7950X (16+ cores, 5.4+ GHz boost)
- **GPU**: NVIDIA RTX 4070 Ti or RTX 4080 (12-16GB VRAM)
  - Full Isaac Sim support with real-time ray tracing
  - Multiple students can develop simultaneously using containerization
- **RAM**: 64GB DDR5 (enables running multiple simulation instances or large ML models)
- **Storage**: 1TB NVMe Gen4 SSD (Isaac Sim assets, datasets, student projects consume significant space)
- **OS**: Ubuntu 22.04 LTS

**Edge Device Kits** (2 kits @ $1,500 each for advanced students):
- **Compute**: NVIDIA Jetson Orin Nano 8GB or Orin NX 16GB
- **Camera**: Intel RealSense Depth Camera D435i ($300)
- **Microphone**: ReSpeaker Mic Array v2.0 for voice commands ($50)
- **IMU**: Adafruit BNO055 9-DOF sensor ($35)
- **Miscellaneous**: Mounts, cables, power bank ($115)

**Optional Robot** (Budget permitting):
- Unitree Go2 Quadruped: $1,800-$3,000 depending on configuration
- Use as proxy for humanoid testing (navigation, perception experiments)
- Lower cost than full humanoid, fewer safety concerns

**Software Stack**:
- All Minimal Tier software +
- NVIDIA Isaac Sim 2023.1.1+ (Omniverse)
- Isaac ROS packages (visual_slam, nvblox, image_pipeline)
- Nav2 navigation stack
- Unity 2022 LTS (optional, for high-fidelity rendering)

**Total Cost**: $10,500 workstations + $3,000 edge kits + $2,500 robot = $16,000 (adjust quantities to meet $15K budget)

**Capabilities**:
- ✅ All 4 modules fully supported without cloud dependency
- ✅ Real-time Isaac Sim rendering and SLAM
- ✅ Optional edge device deployment for advanced students
- ✅ Physical robot testing (if Go2 purchased)

---

### Premium Tier: $50,000+ Budget (Research-Grade Infrastructure)

**Target Use Case**: Graduate programs, research labs, institutions with humanoid robotics research goals

**Workstation Specifications** (5 workstations @ $6,000 each):
- **CPU**: Intel Core i9-14900K or AMD Threadripper (24+ cores)
- **GPU**: NVIDIA RTX 4090 or RTX 6000 Ada (24-48GB VRAM)
- **RAM**: 128GB DDR5
- **Storage**: 2TB NVMe Gen4 SSD in RAID 0

**Edge Device Kits** (3 production-grade @ $3,500 each):
- **Compute**: Jetson AGX Orin 64GB (industrial-grade)
- **Camera**: Intel RealSense D455 (wider FOV than D435i)
- **LiDAR**: SLAMTEC RPLIDAR A3 or Velodyne VLP-16 (robust localization)
- **IMU**: VectorNav VN-100 (industrial INS)
- **Force/Torque Sensors**: ATI Mini40 (for manipulation research)

**Physical Robot Options**:
- **Option A**: Unitree G1 Humanoid Robot (~$16,000) - 23 DoF, 1.32m height, mobile manipulation
- **Option B**: Unitree Go2 Pro + 6-DoF arm (~$8,000 total) - Modular approach

**Lab Infrastructure** ($5,000):
- Safety barriers, motion capture cameras for ground truth, lab benches, power distribution

**Total**: $30,000 workstations + $10,500 edge kits + $16,000 robot + $5,000 lab = $61,500

**Capabilities**:
- ✅ Complete humanoid robotics research pipeline
- ✅ Sim-to-real transfer experiments with quantitative validation
- ✅ Multi-robot coordination (budget allows multiple robots)
- ✅ Publication-quality experimental data
- ✅ Graduate thesis projects in Physical AI

---

**Price Volatility Note**: GPU and robot prices fluctuate significantly. Check vendor sites for current pricing. Links to official sources:
- NVIDIA GPUs: https://www.nvidia.com/en-us/geforce/graphics-cards/
- Jetson modules: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/
- Unitree robotics: https://www.unitree.com/

Principles (budget tiers, simulation-first) remain stable even as specific hardware evolves.
