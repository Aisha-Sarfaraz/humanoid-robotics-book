# Sample Lab Exercises (Continued)

### Lab Exercise 3: Isaac ROS Visual SLAM (Week 9, Module 3)

**Difficulty**: Advanced | **Time**: 6 hours | **Hardware**: Recommended Tier (RTX 4070 Ti minimum)

**Learning Objectives**:
- Install and configure Isaac ROS packages
- Run Visual SLAM on recorded data
- Evaluate SLAM accuracy with quantitative metrics

**Instructions**:
1. Install Isaac ROS: `sudo apt install ros-humble-isaac-ros-visual-slam`
2. Download sample rosbag: [RealSense D435i dataset]
3. Launch SLAM: `ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py`
4. Play rosbag: `ros2 bag play sample_data.bag`
5. Visualize in RViz: trajectory, point cloud, camera poses
6. Compute metrics: ATE (Absolute Trajectory Error), RPE (Relative Pose Error)

**Expected Output**:
- Real-time trajectory visualization in RViz
- ATE < 0.1 meters on provided test dataset
- RPE < 0.05 meters/frame

**Assessment Rubric**:
- SLAM runs without errors (20%)
- Trajectory visualization correct (20%)
- Quantitative metrics computed (30%)
- Analysis of failure cases (30%)

**Common Issues**:
- GPU out of memory → Reduce image resolution
- TF errors → Check camera calibration parameters
- Poor tracking → Increase feature threshold

### Lab Exercise 4: Voice-Controlled Navigation (Week 12, Module 4 - Capstone)

**Difficulty**: Advanced | **Time**: 10-12 hours | **Hardware**: Recommended or Premium Tier

**Learning Objectives**:
- Integrate speech recognition (Whisper), LLM planning (GPT-4), navigation (Nav2), and perception (Isaac ROS)
- Implement closed-loop control with error handling
- Demonstrate complete voice-to-action pipeline

**System Architecture**:
```
Voice Input → Whisper (speech-to-text) → GPT-4 (task planning) → ROS 2 Action Server
                                                                           ↓
                                                              Nav2 (navigation) + Isaac ROS (perception)
                                                                           ↓
                                                              Task Execution + Feedback
```

**Test Cases** (Students must pass 4/5 to demonstrate proficiency):
1. "Go to the kitchen" → Robot navigates to kitchen waypoint
2. "Find the red cup" → Robot detects red cup using object detection
3. "Bring me the cup" → Complete pick-and-place sequence
4. "Go back to start" → Return to origin position
5. "What do you see?" → Report detected objects via text-to-speech

**Assessment Rubric** (20% each):
- Voice recognition accuracy (90%+ word accuracy)
- Task planning correctness (valid action sequences)
- Navigation success rate (reaches goals without collision)
- Perception accuracy (correctly identifies target objects)
- System integration and error handling (graceful failures, informative messages)

**Bonus Points** (Optional):
- Multi-step commands: "Go to kitchen, then go to bedroom"
- Replanning when obstacles detected
- Natural language error reporting: "I cannot find the red cup, but I see a blue cup"

This capstone exercise synthesizes all four modules and represents the culmination of the 13-week curriculum.
