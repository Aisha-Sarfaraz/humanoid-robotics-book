# 4.5 Module 4: Vision-Language-Action (VLA) Models - Weeks 11-12

The final module integrates voice commands, large language models, and robot actions into cohesive systems that respond to natural language. This represents the cutting edge of Physical AI, combining foundation models with embodied intelligence.

## Week 11: Voice-to-Action Pipeline

**Learning Objectives:**
- Integrate speech recognition (Whisper API) with ROS 2
- Parse natural language commands into structured robot tasks
- Handle ambiguity and error cases in language understanding

**Topics:**
- OpenAI Whisper for speech-to-text conversion
- Prompt engineering for robotics tasks
- Command grounding: mapping "kitchen" to map coordinates
- Error handling: clarification dialogs when commands are ambiguous

**Lab Exercise**: Students implement a voice command interface where spoken input ("Go to the kitchen and grab the red cup") is transcribed, parsed into subtasks (navigate to kitchen, identify red cup, grasp cup), and executed by their simulated robot from previous modules.

**Technical Integration**: This exercise synthesizes all prior modules:
- ROS 2 (communication infrastructure from Module 1)
- Gazebo/Unity (virtual environment from Module 2)
- Nav2 (navigation to kitchen from Module 3)
- Vision models (identifying red cup, new in Module 4)

## Week 12: Cognitive Planning with LLMs

**Learning Objectives:**
- Use large language models for high-level task planning
- Translate LLM-generated plans to ROS 2 action sequences
- Implement closed-loop control with perception feedback

**Topics:**
- GPT-4 API for task decomposition
- Converting natural language plans to executable primitives (navigate, grasp, place)
- Visual grounding: using object detection to locate "red cup" from camera images
- Replanning when initial plans fail (cup not in expected location)

**Lab Exercise (Capstone Integration)**: Students build an autonomous humanoid system demonstrating complete voice-to-action workflow:

1. **Voice Input**: User says "Go to the kitchen and grab the red cup"
2. **Speech Recognition**: Whisper transcribes to text
3. **Task Planning**: GPT-4 decomposes into: [navigate(kitchen), detect(red cup), grasp(cup), navigate(start), place(cup)]
4. **Navigation**: Nav2 plans path to kitchen
5. **Perception**: Isaac ROS object detection identifies red cup location
6. **Manipulation**: (Simplified) grasp action or waypoint approach
7. **Feedback**: System reports "Task complete" via text-to-speech

**Assessment Rubric**: Equal weighting (20% each) for:
- Voice recognition accuracy (transcription correctness)
- Task planning correctness (LLM generates valid action sequences)
- Navigation success rate (robot reaches kitchen reliably)
- Object identification accuracy (correctly identifies red cup)
- System integration and error handling (graceful failure recovery)

## Week 13: Final Project Presentations

Students present capstone projects demonstrating mastery of the complete stack. Example projects:

- **Autonomous Cleaning Robot**: Navigates office environment, identifies trash, plans coverage path
- **Warehouse Pick-and-Place**: Receives inventory requests via voice, navigates to shelf, identifies items, returns to packing station
- **Hospital Assistant**: Responds to nurse requests, delivers medication to patient rooms, handles dynamic obstacle avoidance

**Presentation Format**: 10-minute demo + 5 minutes Q&A. Projects must demonstrate integration across all four modules, with emphasis on system robustness and error handling.
