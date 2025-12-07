# Appendices

## Appendix A: Course Syllabus Template

**Course Title**: Physical AI & Humanoid Robotics (CS/ROB XXX)
**Credits**: 3
**Prerequisites**: Programming proficiency (Python), linear algebra, basic probability
**Instructor**: [Name], [Email], [Office Hours]

### Course Description

Hands-on introduction to Physical AI and humanoid robotics using simulation-first approaches. Students learn ROS 2, physics simulation (Gazebo/Isaac), autonomous navigation, and vision-language-action integration. Final project demonstrates voice-controlled robot system.

### Learning Outcomes

By course end, students will:
1. Design and implement ROS 2 systems for robotic applications
2. Develop algorithms in physics-accurate simulation environments
3. Configure autonomous navigation using SLAM and path planning
4. Integrate language models with robotic action execution
5. Evaluate safety, ethics, and societal impact of autonomous systems

### Weekly Schedule (13 Weeks)

| Week | Topics | Assignments |
|------|--------|-------------|
| 1-2 | Foundations: Python, Linux, ROS 2 basics | Lab 1: Setup environment |
| 3 | ROS 2 Nodes and Topics | Lab 2: Talker-Listener |
| 4 | ROS 2 Actions and Services | Lab 3: Voice command node |
| 5 | URDF and Coordinate Transforms | Lab 4: Robot modeling |
| 6 | Gazebo Physics Simulation | Lab 5: Sensor simulation |
| 7 | Unity Integration | Midterm Exam |
| 8 | Isaac Sim Setup | Lab 6: Synthetic data generation |
| 9 | Isaac ROS Visual SLAM | Lab 7: SLAM accuracy metrics |
| 10 | Nav2 Autonomous Navigation | Lab 8: Navigation tuning |
| 11 | Voice-Language-Action Pipeline | Lab 9: Whisper integration |
| 12 | LLM Task Planning | Capstone development |
| 13 | Final Presentations | Capstone due |

### Grading

- Weekly Labs (40%): Hands-on coding exercises
- Midterm Exam (20%): ROS 2 fundamentals, simulation concepts
- Capstone Project (30%): Integrated voice-controlled system
- Participation & Peer Review (10%)

### Required Materials

- Personal laptop (8GB+ RAM, Ubuntu 22.04 or VM)
- Cloud computing credits (AWS Educate or equivalent) for Module 3
- No textbook required (all readings online)

### Academic Integrity

All code must be original or properly attributed. Collaboration encouraged on concepts, but submissions must be individual work unless explicitly group projects.

### Accessibility

All core assignments completable remotely via simulation. Students needing accommodations should contact [Disability Services] within first week.

---

## Appendix B: Sample Assignment Rubrics

### Rubric 1: Weekly Lab Assignments

| Criterion | Excellent (4) | Good (3) | Satisfactory (2) | Needs Work (1) |
|-----------|---------------|----------|------------------|----------------|
| **Functionality** | All requirements met, robust to edge cases | Core functionality works, minor issues | Basic functionality, several bugs | Incomplete or non-functional |
| **Code Quality** | Well-organized, modular, follows style guide | Readable, mostly modular | Functional but disorganized | Hard to understand, poor structure |
| **Documentation** | Clear README, inline comments, usage examples | README present, some comments | Minimal documentation | No documentation |
| **Testing** | Comprehensive tests, handles errors | Basic tests for core functions | Minimal testing | No tests |

**Minimum Passing**: 2/4 in all categories (8/16 total points)

### Rubric 2: Capstone Project

| Criterion | Weight | Excellent (90-100%) | Good (75-89%) | Satisfactory (60-74%) | Needs Work (&lt;60%) |
|-----------|--------|---------------------|---------------|----------------------|-------------------|
| **Voice Recognition** | 20% | 95%+ accuracy, multiple accents | 80-95% accuracy | 60-80% accuracy | &lt;60% or frequent failures |
| **Task Planning** | 20% | Complex multi-step plans | Standard tasks reliable | Simple tasks only | Plans often fail |
| **Navigation** | 20% | 90%+ success rate | 75-90% success | 60-75% success | &lt;60% collision-prone |
| **Perception** | 20% | Robust object detection | Detects in good conditions | Limited scenarios | Unreliable |
| **Integration** | 20% | Graceful error handling | Basic error recovery | Crashes but recoverable | Frequent failures |

**Minimum Passing**: 60% average across all criteria

---

## Appendix C: Hardware Vendor List

### Computing Hardware

**GPU Workstations**:
- Dell Precision 7920 Tower: https://www.dell.com/en-us/work/shop/workstations/precision-7920-tower/
- HP Z8 G4 Workstation: https://www.hp.com/us-en/workstations/z8.html
- Custom builds: Verify Ubuntu 22.04 compatibility before purchase

**NVIDIA GPUs**:
- Official store: https://www.nvidia.com/en-us/geforce/graphics-cards/
- Check real-time stock: https://www.nowinstock.net/computers/videocards/nvidia/

**Edge Devices**:
- NVIDIA Jetson: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/
- RealSense Cameras: https://www.intelrealsense.com/

### Robots

**Unitree Robotics**:
- Official site: https://www.unitree.com/
- Go2 Quadruped: $1,800-$3,000
- G1 Humanoid: ~$16,000
- Contact: sales@unitree.com

**Alternative Platforms**:
- TurtleBot4 (educational wheeled robot): https://clearpathrobotics.com/turtlebot-4/
- Clearpath Robotics (research platforms): https://clearpathrobotics.com/

### Sensors & Components

**Cameras**:
- Intel RealSense D435i/D455: https://www.intelrealsense.com/
- Oak-D cameras: https://store.opencv.ai/

**LiDAR**:
- SLAMTEC RPLiDAR: https://www.slamtec.com/en/Lidar
- Velodyne (research-grade): https://velodynelidar.com/

**Microphones**:
- ReSpeaker Arrays: https://www.seeedstudio.com/

**IMUs**:
- Adafruit sensors: https://www.adafruit.com/category/41
- VectorNav (industrial): https://www.vectornav.com/

### Educational Discounts

- **AWS Educate**: Free cloud credits for students - https://aws.amazon.com/education/awseducate/
- **GitHub Student Pack**: Free tools and services - https://education.github.com/pack
- **NVIDIA Academic Programs**: Educational discounts on Jetson - https://developer.nvidia.com/academic_program

---

## Appendix D: Recommended Readings

### Foundational Robotics

1. Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer Handbook of Robotics* (2nd ed.). Springer. [Comprehensive reference]

2. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press. [Classic text on SLAM, filtering]

### ROS 2 and Modern Frameworks

3. Official ROS 2 Documentation: https://docs.ros.org/en/humble/ [Primary reference]

4. Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming Robots with ROS*. O'Reilly. [ROS 1, but concepts transfer]

### Computer Vision for Robotics

5. Szeliski, R. (2022). *Computer Vision: Algorithms and Applications* (2nd ed.). Springer. [Free online]

### Educational Robotics

6. Benitti, F. B. V. (2012). Exploring the educational potential of robotics in schools: A systematic review. *Computers & Education*, 58(3), 978-988. [Pedagogical research]

### Ethics and Society

7. Calo, R., Froomkin, A. M., & Kerr, I. (Eds.). (2016). *Robot Law*. Edward Elgar Publishing. [Legal frameworks]

8. Lin, P., Abney, K., & Jenkins, R. (Eds.). (2017). *Robot Ethics 2.0*. Oxford University Press. [Philosophical perspectives]

---

## Appendix E: Community Resources

### Online Learning Platforms

- **The Construct**: ROS 2 courses and simulations - https://www.theconstructsim.com/
- **Udemy**: ROS 2 for Beginners - Search "ROS 2" for current offerings
- **Coursera**: Robotics Specialization (University of Pennsylvania)

### Forums and Discussion

- **ROS Discourse**: Official ROS community - https://discourse.ros.org
- **r/ROS subreddit**: Community Q&A - https://reddit.com/r/ROS
- **r/robotics**: General robotics discussion
- **NVIDIA Isaac Sim Forums**: https://forums.developer.nvidia.com/c/isaac-sim/

### Conferences (Student Registration Discounts)

- **ICRA** (IEEE International Conference on Robotics and Automation) - Annual, May/June
- **IROS** (IEEE/RSJ Intelligent Robots and Systems) - Annual, September/October
- **CoRL** (Conference on Robot Learning) - Annual, November
- **RSS** (Robotics: Science and Systems) - Annual, July

### Professional Societies

- **IEEE Robotics and Automation Society**: https://www.ieee-ras.org/
  - Student membership: $32/year
  - Access to IEEE Xplore database
- **ACM SIGAI**: Special Interest Group on Artificial Intelligence

### Open Source Projects

- **Awesome Robotics**: Curated list - https://github.com/kiloreux/awesome-robotics
- **Awesome ROS 2**: https://github.com/fkromer/awesome-ros2
- **Isaac ROS**: NVIDIA's open-source packages - https://github.com/NVIDIA-ISAAC-ROS

### YouTube Channels

- **Articulated Robotics**: ROS 2 tutorials
- **TheConstructSim**: ROS development
- **Boston Dynamics**: Inspiring robot demonstrations

---

## Appendix F: Troubleshooting Common Issues

### ROS 2 Installation Problems

**Issue**: `ros2: command not found`
**Solution**: Source setup file: `source /opt/ros/humble/setup.bash` (add to ~/.bashrc for persistence)

**Issue**: Package build fails with Python import errors
**Solution**: Verify Python 3.10 and install dependencies: `rosdep install --from-paths src --ignore-src -r -y`

### Gazebo Simulation Issues

**Issue**: Gazebo crashes on launch
**Solution**: Update graphics drivers, reduce physics complexity, or allocate more RAM

**Issue**: Robot falls through ground
**Solution**: Check collision geometry in URDF, verify physics parameters (friction, gravity)

### Isaac Sim Issues

**Issue**: "NVIDIA driver version XXX does not meet minimum requirement"
**Solution**: Update NVIDIA drivers to 525+ for RTX support

**Issue**: Out of GPU memory errors
**Solution**: Reduce scene complexity, lower rendering resolution, or use cloud instance with more VRAM

### General Debugging Tips

1. **Use RViz** to visualize topics, transforms, and robot states
2. **Check logs**: `ros2 topic echo /rosout` for error messages
3. **Verify TF tree**: `ros2 run tf2_tools view_frames`
4. **Test incrementally**: Add complexity gradually, verify each component works before integration

---

**End of Appendices**

Return to [Table of Contents](./intro) or continue to [References](./references).
