# Module 4: The Cognitive Layer - Vision-Language-Action (VLA) Models

## 📚 Learning Path Overview

**Duration**: 3 weeks
**Difficulty**: Advanced
**Prerequisites**: Modules 1-3 (ROS 2, Simulation, NVIDIA Isaac)

### Learning Progression
```
Week 1: Vision Foundation Models
  └─> CLIP, Visual Transformers, Object Recognition
       └─> Week 2: Language Models for Robotics
            └─> LLM Task Planning, Natural Language Interface
                 └─> Week 3: VLA Integration
                      └─> Multimodal Fusion, End-to-End Control
```

### Learning Objectives
By the end of this module, you will:
- ✅ Integrate vision foundation models (CLIP, DINO) with ROS 2
- ✅ Use LLMs for high-level task planning and decomposition
- ✅ Build natural language interfaces for robot control
- ✅ Implement end-to-end VLA pipelines (RT-1/RT-2 style)
- ✅ Handle multimodal sensor fusion (vision + language + proprioception)
- ✅ Deploy vision-language models with TensorRT optimization

---

## Week 1: Vision Foundation Models for Robotics

### 🎯 Learning Objectives
- Understand vision transformers and contrastive learning (CLIP)
- Implement visual grounding for natural language queries
- Integrate vision models with ROS 2 perception stack
- Optimize vision models for real-time robot control

### 📖 Theory: From Computer Vision to Embodied Vision

**Traditional Computer Vision vs. Foundation Models**

Traditional robotics vision relied on task-specific models:
- Object detection: Trained on specific object classes
- Semantic segmentation: Fixed categories (person, car, chair)
- Pose estimation: Predefined keypoint sets

**Vision Foundation Models** change the paradigm:
- **CLIP** (Contrastive Language-Image Pre-training): Zero-shot object recognition using natural language descriptions
- **DINO** (Self-Distillation with No Labels): Self-supervised visual features without human labels
- **SAM** (Segment Anything Model): Universal segmentation with prompt engineering

**Key Insight**: Instead of training a new model to detect "red cups," you can query CLIP with "a red cup on a table" and get semantic similarity scores.

**Architecture Overview - CLIP**:
```
Text: "a red cup"  ──┐
                     │
Image: [RGB frame]  ──┼──> Cosine Similarity ──> Match Score
                     │
Vision Encoder       Text Encoder
(ViT-B/32)          (Transformer)
```

**Why This Matters for Robotics**:
- **Open vocabulary**: Recognize objects not in training set
- **Language grounding**: Natural language queries instead of class IDs
- **Faster deployment**: No need to collect/label robot-specific datasets
- **Multimodal reasoning**: Bridge vision and language for task understanding

### 💻 Code Example 1: CLIP-Based Visual Grounding for Robot Perception

This ROS 2 node uses OpenAI's CLIP to perform zero-shot object detection from natural language queries.

```python
#!/usr/bin/env python3
"""
CLIP Visual Grounding Node
Subscribes to camera images, uses CLIP to find objects matching text queries.
Publishes detected object locations for manipulation or navigation.

Dependencies:
  pip install transformers torch torchvision pillow
  sudo apt install ros-humble-cv-bridge ros-humble-vision-msgs
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from transformers import CLIPProcessor, CLIPModel
from PIL import Image as PILImage

class CLIPGroundingNode(Node):
    def __init__(self):
        super().__init__('clip_grounding_node')

        # Parameters
        self.declare_parameter('model_name', 'openai/clip-vit-base-patch32')
        self.declare_parameter('text_queries', ['a red cup', 'a blue bottle', 'a laptop'])
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('grid_size', 16)  # Divide image into 16x16 grid

        model_name = self.get_parameter('model_name').value
        self.text_queries = self.get_parameter('text_queries').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.grid_size = self.get_parameter('grid_size').value

        # Load CLIP model
        self.get_logger().info(f'Loading CLIP model: {model_name}')
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = CLIPModel.from_pretrained(model_name).to(self.device)
        self.processor = CLIPProcessor.from_pretrained(model_name)
        self.model.eval()

        # Precompute text embeddings for queries
        self.text_features = self._encode_text_queries(self.text_queries)

        # ROS 2 setup
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/clip/detections', 10
        )

        self.get_logger().info('CLIP Grounding Node initialized')
        self.get_logger().info(f'Queries: {self.text_queries}')

    def _encode_text_queries(self, queries):
        """Precompute and cache text embeddings"""
        inputs = self.processor(text=queries, return_tensors="pt", padding=True)
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        with torch.no_grad():
            text_features = self.model.get_text_features(**inputs)
            text_features = text_features / text_features.norm(dim=-1, keepdim=True)

        return text_features

    def image_callback(self, msg):
        """Process incoming camera images"""
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        h, w = cv_image.shape[:2]

        # Sliding window approach: divide image into overlapping crops
        detections = Detection2DArray()
        detections.header = msg.header

        step_size = max(h, w) // self.grid_size
        window_size = step_size * 2  # Overlapping windows

        for y in range(0, h - window_size, step_size):
            for x in range(0, w - window_size, step_size):
                crop = cv_image[y:y+window_size, x:x+window_size]

                # Get CLIP similarity scores for this crop
                similarities = self._compute_similarities(crop)

                # Find best matching query
                max_score_idx = similarities.argmax().item()
                max_score = similarities[max_score_idx].item()

                if max_score > self.confidence_threshold:
                    detection = self._create_detection(
                        x, y, window_size, window_size,
                        self.text_queries[max_score_idx],
                        max_score
                    )
                    detections.detections.append(detection)

        # Apply non-maximum suppression to remove overlapping detections
        detections.detections = self._nms(detections.detections, iou_threshold=0.5)

        self.detection_pub.publish(detections)
        self.get_logger().info(f'Published {len(detections.detections)} detections')

    def _compute_similarities(self, crop_np):
        """Compute CLIP similarity between crop and text queries"""
        # Convert numpy to PIL
        crop_pil = PILImage.fromarray(crop_np)

        # Process image
        inputs = self.processor(images=crop_pil, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        # Get image features
        with torch.no_grad():
            image_features = self.model.get_image_features(**inputs)
            image_features = image_features / image_features.norm(dim=-1, keepdim=True)

        # Compute cosine similarity
        similarities = (image_features @ self.text_features.T).squeeze(0)
        return similarities

    def _create_detection(self, x, y, width, height, label, score):
        """Create Detection2D message"""
        detection = Detection2D()

        # Bounding box center
        detection.bbox.center.position.x = float(x + width / 2)
        detection.bbox.center.position.y = float(y + height / 2)
        detection.bbox.size_x = float(width)
        detection.bbox.size_y = float(height)

        # Hypothesis (label + confidence)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = label
        hypothesis.hypothesis.score = float(score)
        detection.results.append(hypothesis)

        return detection

    def _nms(self, detections, iou_threshold=0.5):
        """Non-maximum suppression to remove overlapping detections"""
        if len(detections) == 0:
            return detections

        # Extract bounding boxes and scores
        boxes = []
        scores = []
        for det in detections:
            x = det.bbox.center.position.x - det.bbox.size_x / 2
            y = det.bbox.center.position.y - det.bbox.size_y / 2
            boxes.append([x, y, x + det.bbox.size_x, y + det.bbox.size_y])
            scores.append(det.results[0].hypothesis.score)

        boxes = np.array(boxes)
        scores = np.array(scores)

        # Apply NMS
        indices = self._nms_boxes(boxes, scores, iou_threshold)
        return [detections[i] for i in indices]

    def _nms_boxes(self, boxes, scores, iou_threshold):
        """Pure NumPy NMS implementation"""
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            # Compute IoU with remaining boxes
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            inter = w * h

            iou = inter / (areas[i] + areas[order[1:]] - inter)

            # Keep boxes with IoU below threshold
            order = order[np.where(iou <= iou_threshold)[0] + 1]

        return keep

def main(args=None):
    rclpy.init(args=args)
    node = CLIPGroundingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Technical Details**:
- **Sliding window**: Divides image into overlapping crops to localize objects
- **Precomputed text embeddings**: Encodes text queries once for efficiency
- **Non-maximum suppression**: Removes duplicate detections
- **Zero-shot**: No training required, works with any text description

**Launch this node**:
```bash
ros2 run my_robot_pkg clip_grounding_node --ros-args \
  -p text_queries:="['a red cup', 'a person', 'a door handle']" \
  -p confidence_threshold:=0.3
```

### 🔬 Lab Exercise 1: Visual Question Answering

**Task**: Extend the CLIP node to answer visual questions like:
- "Is there a red object on the table?"
- "How many cups are visible?"
- "What color is the nearest bottle?"

**Hints**:
- Use multiple text prompts: "a red object", "a blue object", etc.
- Count detections above threshold for "how many" questions
- Use spatial reasoning: nearest object = largest bounding box area

### ✅ Assessment Checklist - Week 1

- [ ] CLIP model loads and processes images in under 200ms
- [ ] Visual grounding detects objects with over 70% accuracy on test queries
- [ ] Non-maximum suppression correctly removes duplicate detections
- [ ] Node publishes Detection2DArray messages at camera frame rate
- [ ] Zero-shot queries work without retraining (test with novel objects)

---

## Week 2: Language Models for Task Planning

### 🎯 Learning Objectives
- Use LLMs to decompose high-level commands into robot primitives
- Implement prompt engineering for robotics tasks
- Build natural language command parsing with error handling
- Translate LLM outputs to ROS 2 action sequences

### 📖 Theory: LLMs as Robot Task Planners

**The Challenge of Natural Language Commands**

Traditional robot programming:
```python
robot.navigate_to('kitchen')
robot.detect_object('red_cup')
robot.grasp('red_cup')
```

Natural language request:
> "Hey robot, could you grab me that red cup from the kitchen counter?"

**LLMs bridge this gap** by:
1. **Understanding intent**: "grab" → grasp action
2. **Extracting entities**: "red cup", "kitchen counter"
3. **Task decomposition**: Navigate → Detect → Grasp → Return
4. **Handling ambiguity**: "that red cup" requires visual context

**Prompt Engineering for Robotics**

The quality of LLM output depends heavily on prompt structure:

**Bad Prompt**:
```
User: Go to the kitchen and grab the red cup
```

**Good Prompt**:
```
You are a robot task planner. Decompose the user's request into a sequence
of primitive actions from this list:
- navigate(location: str)
- detect(object: str)
- grasp(object: str)
- place(location: str)
- say(message: str)

User request: "Go to the kitchen and grab the red cup"

Output format (JSON):
{
  "plan": [
    {"action": "navigate", "params": {"location": "kitchen"}},
    {"action": "detect", "params": {"object": "red cup"}},
    ...
  ],
  "assumptions": ["Kitchen location is known in map", "Red cup is visible"],
  "potential_failures": ["Cup not found", "Navigation blocked"]
}
```

**Key Principles**:
- **Constrain output format**: JSON, YAML, or structured text
- **Define primitive actions**: Finite action vocabulary
- **Request assumptions/failures**: Forces reasoning about edge cases
- **Few-shot examples**: Include 2-3 example decompositions

### 💻 Code Example 2: LLM Task Planner for Robot Commands

This node uses OpenAI's GPT-4 to decompose natural language commands into executable ROS 2 action sequences.

```python
#!/usr/bin/env python3
"""
LLM Task Planner Node
Receives natural language commands via service, uses GPT-4 to generate
action plans, and publishes to execution coordinator.

Dependencies:
  pip install openai pydantic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import SetString
import openai
import json
from typing import List, Dict, Optional
from pydantic import BaseModel, Field

# Define structured output schema
class ActionPrimitive(BaseModel):
    action: str = Field(..., description="Action name from predefined set")
    params: Dict[str, str] = Field(..., description="Action parameters")
    timeout: float = Field(10.0, description="Max execution time in seconds")

class TaskPlan(BaseModel):
    plan: List[ActionPrimitive]
    assumptions: List[str]
    potential_failures: List[str]
    estimated_duration: float

class LLMTaskPlanner(Node):
    def __init__(self):
        super().__init__('llm_task_planner')

        # Parameters
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('model', 'gpt-4')
        self.declare_parameter('temperature', 0.2)  # Low temperature for consistent output

        api_key = self.get_parameter('openai_api_key').value
        if not api_key:
            self.get_logger().error('OpenAI API key not provided!')
            return

        openai.api_key = api_key
        self.model = self.get_parameter('model').value
        self.temperature = self.get_parameter('temperature').value

        # Define robot's action primitives
        self.action_primitives = {
            'navigate': 'Move to a named location (params: location)',
            'detect': 'Detect objects matching description (params: object_description)',
            'grasp': 'Grasp detected object (params: object_id)',
            'place': 'Place held object at location (params: location)',
            'say': 'Speak text via TTS (params: message)',
            'wait': 'Wait for duration (params: duration_sec)',
        }

        # ROS 2 interfaces
        self.plan_service = self.create_service(
            SetString, '/robot/plan_task', self.plan_task_callback
        )
        self.plan_pub = self.create_publisher(String, '/robot/task_plan', 10)

        self.get_logger().info('LLM Task Planner initialized')
        self.get_logger().info(f'Available actions: {list(self.action_primitives.keys())}')

    def plan_task_callback(self, request, response):
        """Service callback: generate plan from natural language command"""
        user_command = request.data
        self.get_logger().info(f'Planning task: "{user_command}"')

        try:
            plan = self._generate_plan(user_command)
            response.success = True
            response.message = json.dumps(plan.dict(), indent=2)

            # Publish plan for execution
            plan_msg = String()
            plan_msg.data = response.message
            self.plan_pub.publish(plan_msg)

            self.get_logger().info(f'Generated plan with {len(plan.plan)} steps')
        except Exception as e:
            response.success = False
            response.message = f'Planning failed: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def _generate_plan(self, user_command: str) -> TaskPlan:
        """Use GPT-4 to decompose command into action sequence"""

        # Construct system prompt
        system_prompt = self._build_system_prompt()

        # Construct user prompt with few-shot examples
        user_prompt = f"""
User command: "{user_command}"

Generate a task plan following the output schema.
"""

        # Call OpenAI API
        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": "Go to the living room and turn on the lights"},
                {"role": "assistant", "content": self._example_plan_1()},
                {"role": "user", "content": user_prompt}
            ],
            temperature=self.temperature,
            max_tokens=500
        )

        # Parse response
        plan_text = response.choices[0].message.content
        plan_dict = json.loads(plan_text)
        plan = TaskPlan(**plan_dict)

        # Validate all actions are in primitive set
        for step in plan.plan:
            if step.action not in self.action_primitives:
                raise ValueError(f'Invalid action: {step.action}')

        return plan

    def _build_system_prompt(self) -> str:
        """Construct system prompt defining robot's capabilities"""
        actions_desc = '\n'.join([
            f"- {name}: {desc}" for name, desc in self.action_primitives.items()
        ])

        return f"""You are a task planning system for a mobile manipulator robot.

Your job is to decompose natural language commands into sequences of primitive actions.

Available primitive actions:
{actions_desc}

Output format (JSON):
{{
  "plan": [
    {{"action": "navigate", "params": {{"location": "kitchen"}}, "timeout": 30.0}},
    {{"action": "detect", "params": {{"object_description": "red cup"}}, "timeout": 10.0}},
    ...
  ],
  "assumptions": ["List of assumptions about environment/objects"],
  "potential_failures": ["Possible failure modes"],
  "estimated_duration": 45.0
}}

Important:
- Only use actions from the predefined list
- Be specific in object descriptions for detect actions
- Include reasonable timeouts (navigate: 30s, detect: 10s, grasp: 15s)
- List assumptions (e.g., "Kitchen location known in map")
- Consider failure cases (e.g., "Object not found")
- Estimate total duration based on timeouts
"""

    def _example_plan_1(self) -> str:
        """Few-shot example for prompt"""
        return json.dumps({
            "plan": [
                {"action": "navigate", "params": {"location": "living_room"}, "timeout": 30.0},
                {"action": "detect", "params": {"object_description": "light switch"}, "timeout": 10.0},
                {"action": "say", "params": {"message": "Lights turned on"}, "timeout": 2.0}
            ],
            "assumptions": [
                "Living room location is in the map",
                "Light switch is visible and detectable",
                "Robot has capability to interact with switches (simplified)"
            ],
            "potential_failures": [
                "Navigation blocked by obstacles",
                "Light switch not detected",
                "Switch already on"
            ],
            "estimated_duration": 42.0
        }, indent=2)

def main(args=None):
    rclpy.init(args=args)
    node = LLMTaskPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Test the planner**:
```bash
# Terminal 1: Run the planner node
ros2 run my_robot_pkg llm_task_planner --ros-args \
  -p openai_api_key:=$OPENAI_API_KEY

# Terminal 2: Send a planning request
ros2 service call /robot/plan_task example_interfaces/srv/SetString \
  "{data: 'Go to the kitchen and grab the red cup from the counter'}"
```

### 💻 Code Example 3: Natural Language Command Parser with Error Handling

This node adds conversational error handling and clarification dialogs.

```python
#!/usr/bin/env python3
"""
Conversational Command Parser
Handles ambiguous commands by asking clarification questions.

Example:
  User: "Grab the cup"
  Robot: "I see 3 cups - red, blue, and green. Which one?"
  User: "The red one"
  Robot: "Got it, executing: grasp red cup"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import SetString
import openai
import json
from enum import Enum

class ConversationState(Enum):
    IDLE = 1
    AWAITING_CLARIFICATION = 2
    EXECUTING = 3

class ConversationalParser(Node):
    def __init__(self):
        super().__init__('conversational_parser')

        # Parameters
        self.declare_parameter('openai_api_key', '')
        api_key = self.get_parameter('openai_api_key').value
        openai.api_key = api_key

        # Conversation state
        self.state = ConversationState.IDLE
        self.conversation_history = []
        self.pending_command = None
        self.detected_objects = []  # Simulated perception state

        # ROS 2 interfaces
        self.command_service = self.create_service(
            SetString, '/robot/voice_command', self.command_callback
        )
        self.response_pub = self.create_publisher(String, '/robot/response', 10)

        # Subscribe to perception updates
        self.create_subscription(String, '/clip/detected_objects',
                                self.perception_callback, 10)

        self.get_logger().info('Conversational Parser ready')

    def perception_callback(self, msg):
        """Update list of detected objects"""
        self.detected_objects = json.loads(msg.data)

    def command_callback(self, request, response):
        """Process voice command with context awareness"""
        user_input = request.data
        self.get_logger().info(f'Received: "{user_input}"')

        # Add to conversation history
        self.conversation_history.append({"role": "user", "content": user_input})

        try:
            # Check if this is a clarification response
            if self.state == ConversationState.AWAITING_CLARIFICATION:
                result = self._handle_clarification(user_input)
            else:
                result = self._handle_new_command(user_input)

            response.success = result['success']
            response.message = result['message']

            # Publish robot's response
            response_msg = String()
            response_msg.data = result['message']
            self.response_pub.publish(response_msg)

        except Exception as e:
            response.success = False
            response.message = f'Error: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def _handle_new_command(self, command: str) -> dict:
        """Process a new command, checking for ambiguities"""

        # Analyze command with current perception context
        analysis = self._analyze_command_with_context(command)

        if analysis['is_ambiguous']:
            # Need clarification
            self.state = ConversationState.AWAITING_CLARIFICATION
            self.pending_command = command
            return {
                'success': False,
                'message': analysis['clarification_question']
            }
        else:
            # Clear command, can execute
            self.state = ConversationState.EXECUTING
            return {
                'success': True,
                'message': f"Executing: {analysis['action_plan']}"
            }

    def _handle_clarification(self, response: str) -> dict:
        """Process user's clarification response"""

        # Resolve ambiguity using clarification
        resolved_command = self._resolve_ambiguity(self.pending_command, response)

        self.state = ConversationState.IDLE
        self.pending_command = None

        return {
            'success': True,
            'message': f"Got it, executing: {resolved_command}"
        }

    def _analyze_command_with_context(self, command: str) -> dict:
        """Use LLM to analyze command given current perception state"""

        system_prompt = f"""You are a robot command analyzer.
Current visible objects: {json.dumps(self.detected_objects)}

Analyze if the user's command is ambiguous given what the robot can see.

If ambiguous, generate a clarification question.
If clear, generate the action plan.

Output JSON:
{{
  "is_ambiguous": true/false,
  "reason": "explanation",
  "clarification_question": "question to ask user" (if ambiguous),
  "action_plan": "what to execute" (if not ambiguous)
}}
"""

        response = openai.ChatCompletion.create(
            model='gpt-4',
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": command}
            ],
            temperature=0.3
        )

        return json.loads(response.choices[0].message.content)

    def _resolve_ambiguity(self, original_command: str, clarification: str) -> str:
        """Combine original command with clarification"""

        system_prompt = """Combine the original ambiguous command with the user's
clarification into a clear, executable command."""

        user_prompt = f"""
Original: "{original_command}"
Clarification: "{clarification}"

Output the resolved command.
"""

        response = openai.ChatCompletion.create(
            model='gpt-4',
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.2
        )

        return response.choices[0].message.content

def main(args=None):
    rclpy.init(args=args)
    node = ConversationalParser()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Conversation Example**:
```bash
$ ros2 service call /robot/voice_command example_interfaces/srv/SetString \
  "{data: 'Grab the cup'}"

Response: "I see 3 cups - red, blue, and green. Which one should I grab?"

$ ros2 service call /robot/voice_command example_interfaces/srv/SetString \
  "{data: 'The red one near the laptop'}"

Response: "Got it, executing: grasp red cup at (x: 0.45, y: 0.32)"
```

### 🔬 Lab Exercise 2: Multi-Step Task Execution

**Task**: Implement a task executor that runs the LLM-generated plans from Code Example 2.

**Requirements**:
1. Subscribe to `/robot/task_plan` topic
2. For each action in the plan:
   - Call corresponding ROS 2 action server (navigate, detect, grasp)
   - Handle timeouts and failures
   - If step fails, ask LLM to replan from current state
3. Publish progress updates to `/robot/task_progress`

**Hints**:
- Use `rclpy.action.ActionClient` for each primitive action
- Implement error recovery: if `detect` fails, call `navigate` to search location
- Log all steps for debugging

### ✅ Assessment Checklist - Week 2

- [ ] LLM planner decomposes commands into valid action sequences
- [ ] Prompt engineering constrains output to defined primitives
- [ ] Conversational parser handles at least 3 types of ambiguity
- [ ] Error handling gracefully recovers from LLM API failures
- [ ] Task executor successfully runs multi-step plans (over 80% success rate)

---

## Week 3: End-to-End VLA Integration

### 🎯 Learning Objectives
- Understand VLA model architectures (RT-1, RT-2)
- Implement multimodal sensor fusion (vision + language + proprioception)
- Deploy end-to-end VLA policies for robot control
- Evaluate sim-to-real transfer for VLA models

### 📖 Theory: Vision-Language-Action Models

**What is a VLA Model?**

Traditional robotics pipeline:
```
Perception → Task Planning → Motion Planning → Control
(separate models for each stage)
```

VLA models unify this into a single neural network:
```
[Image + Language Command + Robot State]
         ↓
   VLA Transformer
         ↓
   [Action Outputs: joint velocities, gripper state]
```

**Key VLA Architectures**:

**RT-1** (Robotics Transformer 1):
- **Input**: Image (300x300 RGB) + Text command + Robot state (joint angles)
- **Architecture**: Vision Transformer (ViT) for image encoding, concatenated with text embeddings and robot state, fed to Transformer decoder
- **Output**: Discretized action bins (e.g., 256 bins for each joint)
- **Training**: Imitation learning on 130k robot demonstrations

**RT-2** (Robotics Transformer 2):
- Built on PaLM-E (vision-language model)
- **Key innovation**: Pretrains on web-scale vision-language data, fine-tunes on robot data
- **Emergent capabilities**: Generalizes to objects never seen in robot training (zero-shot transfer from internet knowledge)

**Multimodal Fusion Architecture**:
```
┌─────────────┐
│ RGB Camera  │──┐
└─────────────┘  │
┌─────────────┐  │    ┌──────────────┐
│ Depth Cam   │──┼───→│ Vision Encoder│──┐
└─────────────┘  │    └──────────────┘  │
┌─────────────┐  │                      │    ┌─────────────┐
│ Wrist Cam   │──┘                      ├───→│  Transformer│
└─────────────┘                         │    │   Backbone  │──→ Actions
┌─────────────┐        ┌──────────────┐ │    └─────────────┘
│"Pick red cup"│───────→│ Text Encoder │─┤
└─────────────┘        └──────────────┘ │
┌─────────────┐        ┌──────────────┐ │
│ Joint angles│───────→│  MLP Encoder │─┘
│ Gripper state│       └──────────────┘
└─────────────┘
```

**Why VLA Models Matter**:
- **Simplified deployment**: One model instead of perception + planning + control
- **Emergent reasoning**: Language grounding enables zero-shot generalization
- **End-to-end optimization**: Gradients flow through entire pipeline
- **Scalability**: Can leverage massive vision-language datasets

### 💻 Code Example 4: End-to-End VLA Pipeline

This example demonstrates a simplified VLA model for pick-and-place tasks, inspired by RT-1.

```python
#!/usr/bin/env python3
"""
Simplified VLA Model for Robot Control
Integrates vision (camera), language (task description), and proprioception
(robot state) to output low-level actions.

Note: This is a teaching example. Production VLA models like RT-1/RT-2 require
extensive datasets and compute. This demonstrates the architecture.

Dependencies:
  pip install torch torchvision transformers einops
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
import torch.nn as nn
from transformers import CLIPModel, CLIPProcessor
import numpy as np
from typing import Dict, List
import cv2

class VLAModel(nn.Module):
    """
    Vision-Language-Action Transformer

    Architecture:
      1. Vision encoder: CLIP ViT-B/32 (frozen)
      2. Language encoder: CLIP text encoder (frozen)
      3. Proprioception encoder: MLP
      4. Fusion: Concatenate embeddings
      5. Policy head: Transformer → Action outputs
    """

    def __init__(self, action_dim=7, hidden_dim=512):
        super().__init__()

        # Vision-language encoders (pretrained, frozen)
        self.clip_model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
        self.clip_processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")

        # Freeze CLIP weights
        for param in self.clip_model.parameters():
            param.requires_grad = False

        # Proprioception encoder (robot joint states)
        self.proprio_encoder = nn.Sequential(
            nn.Linear(14, hidden_dim),  # 7 joints × 2 (position + velocity)
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim)
        )

        # Fusion and policy network
        # CLIP vision: 512-dim, CLIP text: 512-dim, proprio: 512-dim → total 1536-dim
        self.fusion_dim = 512 * 3

        # Transformer policy head
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=self.fusion_dim,
            nhead=8,
            dim_feedforward=2048,
            batch_first=True
        )
        self.policy_transformer = nn.TransformerEncoder(encoder_layer, num_layers=4)

        # Action head (discretized actions for simplicity)
        self.action_head = nn.Sequential(
            nn.Linear(self.fusion_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim * 256)  # 256 bins per action dimension
        )

        self.action_dim = action_dim

    def forward(self, image, text, proprio_state):
        """
        Args:
            image: (B, 3, H, W) RGB image
            text: List[str] - task descriptions
            proprio_state: (B, 14) joint positions and velocities

        Returns:
            action_logits: (B, action_dim, 256) - discretized action probabilities
        """
        batch_size = image.size(0)
        device = image.device

        # 1. Encode vision
        vision_features = self.clip_model.get_image_features(pixel_values=image)
        # (B, 512)

        # 2. Encode language
        text_inputs = self.clip_processor(text=text, return_tensors="pt", padding=True)
        text_inputs = {k: v.to(device) for k, v in text_inputs.items()}
        text_features = self.clip_model.get_text_features(**text_inputs)
        # (B, 512)

        # 3. Encode proprioception
        proprio_features = self.proprio_encoder(proprio_state)
        # (B, 512)

        # 4. Fuse all modalities
        fused_features = torch.cat([vision_features, text_features, proprio_features], dim=-1)
        # (B, 1536)

        # Add sequence dimension for transformer (treat as single token)
        fused_features = fused_features.unsqueeze(1)  # (B, 1, 1536)

        # 5. Policy transformer
        policy_features = self.policy_transformer(fused_features)
        # (B, 1, 1536)

        policy_features = policy_features.squeeze(1)  # (B, 1536)

        # 6. Action head
        action_logits = self.action_head(policy_features)
        # (B, action_dim * 256)

        # Reshape to (B, action_dim, 256)
        action_logits = action_logits.view(batch_size, self.action_dim, 256)

        return action_logits

    def predict_action(self, image, text, proprio_state):
        """
        Predict continuous actions from discretized logits

        Returns:
            action: (B, action_dim) continuous action values in [-1, 1]
        """
        with torch.no_grad():
            action_logits = self.forward(image, text, proprio_state)

            # Get most likely bin for each action dimension
            action_bins = action_logits.argmax(dim=-1)  # (B, action_dim)

            # Convert bins to continuous values: [0, 255] → [-1, 1]
            actions = (action_bins.float() / 255.0) * 2.0 - 1.0

            return actions

class VLAControlNode(Node):
    """ROS 2 node that runs VLA model for robot control"""

    def __init__(self):
        super().__init__('vla_control_node')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('device', 'cuda')

        model_path = self.get_parameter('model_path').value
        control_freq = self.get_parameter('control_frequency').value
        device = self.get_parameter('device').value

        # Initialize VLA model
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')
        self.model = VLAModel(action_dim=7).to(self.device)

        if model_path:
            self.get_logger().info(f'Loading model from {model_path}')
            self.model.load_state_dict(torch.load(model_path))
        else:
            self.get_logger().warn('No model path provided, using untrained model')

        self.model.eval()

        # ROS 2 state
        self.bridge = CvBridge()
        self.current_image = None
        self.current_joints = None
        self.current_task = "pick up the red block"  # Default task

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.task_sub = self.create_subscription(
            String, '/task_command', self.task_callback, 10
        )

        # Publishers
        self.action_pub = self.create_publisher(JointState, '/vla/joint_commands', 10)

        # Control loop timer
        self.timer = self.create_timer(1.0 / control_freq, self.control_loop)

        self.get_logger().info(f'VLA Control Node initialized on {self.device}')

    def image_callback(self, msg):
        """Store latest camera image"""
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def joint_callback(self, msg):
        """Store latest joint state"""
        # Assume 7-DOF arm
        if len(msg.position) >= 7 and len(msg.velocity) >= 7:
            self.current_joints = {
                'position': np.array(msg.position[:7]),
                'velocity': np.array(msg.velocity[:7])
            }

    def task_callback(self, msg):
        """Update current task description"""
        self.current_task = msg.data
        self.get_logger().info(f'New task: {self.current_task}')

    def control_loop(self):
        """Main control loop: run VLA model and publish actions"""

        # Check if we have all required inputs
        if self.current_image is None or self.current_joints is None:
            return

        try:
            # Prepare inputs
            image_tensor = self._preprocess_image(self.current_image)
            proprio_tensor = self._preprocess_proprioception(self.current_joints)

            # Run VLA model
            actions = self.model.predict_action(
                image_tensor,
                [self.current_task],
                proprio_tensor
            )

            # Publish actions
            self._publish_actions(actions[0].cpu().numpy())

        except Exception as e:
            self.get_logger().error(f'Control loop error: {str(e)}')

    def _preprocess_image(self, image_np):
        """Convert numpy image to model input tensor"""
        # Resize to 224x224 (CLIP standard)
        image_resized = cv2.resize(image_np, (224, 224))

        # Normalize to [0, 1] and convert to tensor
        image_tensor = torch.from_numpy(image_resized).float() / 255.0

        # (H, W, C) → (C, H, W)
        image_tensor = image_tensor.permute(2, 0, 1)

        # Add batch dimension
        image_tensor = image_tensor.unsqueeze(0).to(self.device)

        return image_tensor

    def _preprocess_proprioception(self, joints):
        """Convert joint state dict to tensor"""
        # Concatenate position and velocity
        proprio_np = np.concatenate([joints['position'], joints['velocity']])

        # Normalize (simple scaling, real systems need careful calibration)
        proprio_np = proprio_np / 3.0  # Rough normalization to [-1, 1]

        proprio_tensor = torch.from_numpy(proprio_np).float().unsqueeze(0)
        return proprio_tensor.to(self.device)

    def _publish_actions(self, actions):
        """Publish predicted actions as joint commands"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Scale actions from [-1, 1] to actual joint limits
        # (In real system, use robot-specific scaling)
        scaled_actions = actions * 0.1  # Conservative scaling

        msg.velocity = scaled_actions.tolist()

        self.action_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VLAControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Architecture Points**:
- **Multimodal fusion**: Vision (CLIP), language (CLIP text), proprioception (MLP)
- **Discretized actions**: Actions quantized into 256 bins (reduces distribution complexity)
- **Frozen encoders**: CLIP weights frozen, only policy head trained (transfer learning)
- **Real-time capable**: Forward pass ~50ms on GPU

**Launch VLA control**:
```bash
ros2 run my_robot_pkg vla_control_node --ros-args \
  -p model_path:=/path/to/vla_model.pth \
  -p control_frequency:=20.0
```

### 🔬 Lab Exercise 3: VLA Data Collection and Fine-Tuning

**Task**: Collect demonstration data and fine-tune the VLA model.

**Phase 1: Data Collection**
1. Teleoperate robot in simulation (Gazebo or Isaac)
2. Record episodes: (image, task_description, joint_state, action)
3. Save dataset in HDF5 format:
   ```python
   episode_0/
     ├─ observations/images: (T, 224, 224, 3)
     ├─ observations/proprio: (T, 14)
     ├─ actions: (T, 7)
     └─ language: "pick up red block"
   ```

**Phase 2: Training**
1. Implement training loop:
   ```python
   for batch in dataloader:
       action_logits = model(batch['image'], batch['text'], batch['proprio'])
       loss = F.cross_entropy(action_logits, batch['action_bins'])
       loss.backward()
       optimizer.step()
   ```
2. Train for 50k steps with AdamW optimizer
3. Evaluate on held-out tasks

**Success Metric**: Model achieves over 60% success rate on pick-and-place validation tasks.

### ✅ Assessment Checklist - Week 3

- [ ] VLA model forward pass completes in under 100ms
- [ ] Multimodal fusion correctly combines vision, language, and proprioception
- [ ] Discretized action outputs map to valid continuous actions
- [ ] Data collection pipeline records synchronized observations and actions
- [ ] Fine-tuned model outperforms random baseline by more than 3x
- [ ] Sim-to-real gap analysis completed (domain randomization helps?)

---

## 📝 Module Summary

### Key Takeaways

**Vision Foundation Models**:
- ✅ CLIP enables zero-shot object detection via natural language
- ✅ Sliding window + NMS enables spatial localization
- ✅ Vision transformers outperform CNNs for open-vocabulary tasks

**LLM Task Planning**:
- ✅ Prompt engineering is critical for constraining LLM outputs
- ✅ Few-shot examples dramatically improve plan quality
- ✅ Conversational error handling enables robust human-robot interaction
- ✅ LLMs excel at high-level planning but need grounding in perception

**VLA End-to-End Models**:
- ✅ VLA models unify perception, planning, and control
- ✅ Multimodal fusion (vision + language + proprio) enables emergent reasoning
- ✅ Discretized actions simplify distribution modeling
- ✅ Transfer learning from vision-language models accelerates training

### Skills Mastered

- [ ] Integrated CLIP for visual grounding in ROS 2
- [ ] Implemented LLM-based task planners with structured outputs
- [ ] Built conversational command parsing with error recovery
- [ ] Designed and deployed multimodal VLA architectures
- [ ] Collected robot demonstration data for imitation learning
- [ ] Fine-tuned VLA models on custom tasks

### Common Pitfalls

1. **CLIP Limitations**:
   - Poor at fine-grained spatial reasoning (exact pose estimation)
   - Struggles with occlusions and clutter
   - **Solution**: Combine with traditional detection when precision needed

2. **LLM Hallucinations**:
   - LLMs may generate invalid actions or assume capabilities robot doesn't have
   - **Solution**: Strict output validation, constrain action vocabulary

3. **VLA Data Hunger**:
   - VLA models require thousands of demonstrations
   - **Solution**: Use simulation for data generation, domain randomization

4. **Sim-to-Real Gap**:
   - Models trained in simulation may fail on real robots
   - **Solution**: Visual domain randomization, sensor noise injection, real-world fine-tuning

---

## 🎓 Final Project: Multimodal Voice-Controlled Robot

### Project Requirements

Build an autonomous system that:

1. **Accepts voice commands** (use Whisper for speech-to-text)
2. **Plans tasks with LLM** (GPT-4 task decomposition)
3. **Perceives environment** (CLIP visual grounding)
4. **Executes actions** (VLA model or motion primitives)
5. **Provides feedback** (text-to-speech status updates)

### Example Workflow

```
User: "Go to the kitchen and grab the red mug"
  ↓
[Whisper] Transcribe audio → text
  ↓
[LLM] Plan: [navigate(kitchen), detect(red mug), grasp(mug), navigate(user), place(table)]
  ↓
[Navigation] Move to kitchen (Nav2 + Isaac Sim)
  ↓
[CLIP] Detect "red mug" in camera feed → bounding box
  ↓
[VLA Model] Execute grasp action from visual input
  ↓
[Navigation] Return to user
  ↓
[TTS] "Task complete, here's your red mug"
```

### Evaluation Rubric

| Component | Weight | Criteria |
|-----------|--------|----------|
| **Voice Interface** | 15% | Transcription accuracy over 90%, handles background noise |
| **Task Planning** | 20% | LLM generates valid, safe action sequences |
| **Visual Grounding** | 20% | CLIP correctly identifies target objects over 80% |
| **Action Execution** | 25% | Robot successfully completes pick-and-place |
| **Error Handling** | 10% | Gracefully handles failures (object not found, etc.) |
| **Integration** | 10% | All components work together end-to-end |

### Deliverables

1. **Code**: Complete ROS 2 workspace with all nodes
2. **Video Demo**: 3-minute demonstration of successful task completion
3. **Report**:
   - System architecture diagram
   - Performance metrics (success rate, latency breakdown)
   - Challenges and solutions
   - Future improvements
4. **Presentation**: 10-minute demo + 5-minute Q&A

---

## 📚 Additional Resources

### Papers
- **RT-1**: "RT-1: Robotics Transformer for Real-World Control at Scale" (2022)
- **RT-2**: "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control" (2023)
- **CLIP**: "Learning Transferable Visual Models From Natural Language Supervision" (2021)
- **PaLM-E**: "PaLM-E: An Embodied Multimodal Language Model" (2023)

### Code Repositories
- **OpenAI CLIP**: https://github.com/openai/CLIP
- **Robotics Transformer (RT-1)**: https://github.com/google-research/robotics_transformer
- **Hugging Face Transformers**: https://github.com/huggingface/transformers

### Datasets
- **Open X-Embodiment**: Large-scale robot manipulation dataset (800k+ trajectories)
- **RoboNet**: Multi-robot dataset for visual imitation learning
- **CALVIN**: Benchmark for language-conditioned manipulation

### Tools
- **Weights & Biases**: Experiment tracking for VLA training
- **TensorBoard**: Visualize training metrics
- **Rerun**: Visualize multimodal robot data (images + poses + text)

### Online Courses
- **Stanford CS336**: Robot Learning (focuses on VLA models)
- **DeepMind x UCL**: Deep Learning Lecture on Multimodal Models
- **Hugging Face Course**: Transformers and Vision-Language Models

---

## 🔗 Integration with Previous Modules

**Module 1 (ROS 2)**:
- CLIP node publishes `Detection2DArray` messages
- LLM planner uses ROS 2 services for task requests
- VLA model subscribes to joint states and camera feeds

**Module 2 (Simulation)**:
- Gazebo/Unity provides synthetic data for VLA training
- Domain randomization improves sim-to-real transfer
- Photorealistic Unity rendering helps CLIP generalize

**Module 3 (NVIDIA Isaac)**:
- Isaac ROS DOPE for 6D pose estimation (complements CLIP)
- TensorRT optimization for VLA model inference
- Isaac Sim synthetic data generation for VLA datasets

**Complete Tech Stack**:
```
Voice Input (Whisper)
    ↓
LLM Planning (GPT-4)
    ↓
Visual Grounding (CLIP) ─────┐
    ↓                        │
VLA Model (RT-1/RT-2) ←──────┤
    ↓                        │
ROS 2 Control (Module 1) ────┤
    ↓                        │
Simulation (Module 2) ────────┤
    ↓                        │
Isaac GPU Accel (Module 3) ───┘
    ↓
Physical Robot
```

---

**Congratulations!** You've completed the VLA module and now have the skills to build cognitive, language-grounded robots that understand and execute natural language commands. This represents the cutting edge of Physical AI research.

**Next**: Move on to the remaining chapters to explore real-world deployment, ethics, and advanced topics in humanoid robotics.
