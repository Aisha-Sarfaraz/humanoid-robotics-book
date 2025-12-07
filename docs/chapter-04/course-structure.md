# Chapter 4: Curriculum Design Framework

## Overview

This chapter presents a complete 13-week curriculum framework for teaching Physical AI and Humanoid Robotics in university settings. The framework is modular, simulation-first, and designed for adaptation to quarter (10-week), semester (15-week), or intensive (2-4 week) formats.

**Learning Objectives:**
- Understand the overall course structure and learning progression
- Map weekly topics to specific learning outcomes and assessment criteria
- Adapt the framework to institutional constraints (term length, student backgrounds)
- Design assessments aligned with industry skill requirements

**Estimated Reading Time:** 15-18 minutes

---

## 4.1 Course Structure Overview

The curriculum follows a **progressive complexity model**, beginning with foundational concepts and building toward integrated systems capstone projects. The 13-week structure accommodates standard academic terms while providing buffer time for debugging, review, and final presentations.

### Four-Module Architecture

The course divides into four major modules, each 2-3 weeks in duration:

**Module 1: ROS 2 Fundamentals** (Weeks 3-5) - The robotic nervous system: communication infrastructure, message passing, coordinate transformations

**Module 2: Physics Simulation** (Weeks 6-7) - The digital twin: virtual environments for testing algorithms before hardware deployment

**Module 3: NVIDIA Isaac Platform** (Weeks 8-10) - The AI-robot brain: hardware-accelerated perception, SLAM, and autonomous navigation

**Module 4: Vision-Language-Action Models** (Weeks 11-12) - Cognitive layer: translating natural language commands to robot actions

This architecture reflects modern industry robotics stacks where ROS 2 provides middleware, simulation validates designs, Isaac accelerates perception, and foundation models enable natural human-robot interaction.

### Learning Progression Strategy

The curriculum follows three pedagogical principles:

**1. Concrete Before Abstract**: Students manipulate working code (modify example ROS nodes) before implementing from scratch, providing mental models for abstract concepts.

**2. 2D Before 3D**: Navigation begins with wheeled robots in planar environments before tackling bipedal humanoid locomotion, reducing cognitive load.

**3. Simulation Before Hardware**: All algorithms work reliably in virtual environments before optional physical deployment, separating software debugging from hardware troubleshooting.

### Weekly Time Allocation

Each week assumes **9-12 hours of student effort** (3 hours lecture/discussion, 6-9 hours lab work):

- **Lecture/Discussion** (3 hours): Concept introduction, worked examples, Q&A
- **Guided Lab** (3 hours): Structured exercises with instructor support
- **Independent Project Work** (3-6 hours): Students apply concepts to weekly assignments or capstone development

This workload aligns with standard 3-credit university courses while accommodating the intensive hands-on nature of robotics education.
