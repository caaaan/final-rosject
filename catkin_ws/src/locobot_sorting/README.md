# Robot Arm Object Sorting System

## Overview

This project implements a control pipeline for a robot arm to autonomously sort objects based on their color (specifically red or blue). The robot arm detects objects using a camera, classifies them by color, and then moves each object to its respective sorting mat.

The key components of this project include object detection using camera input, sorting logic based on object properties, and robotic arm control to execute grasping and placement actions.

the github of this project: https://github.com/caaaan/final-rosject
## Project Structure

The project consists of the following main components:
1. **Node for Object Detection** (`camera_object_detection.py`)
2. **Node for Sorting** (`object_sorter_grasp.py`)
3. **Object Detection Utilities** (`object_detection_utils.py`)
4. **Simulation Setup and Launch** (`setup_simulation.launch`)

### 1. CameraObjectDetection Node
- **Purpose**: Detects objects using camera data and classifies them by color.
- **Functionality**: 
  - Captures camera frames and processes them to identify objects.
  - Classifies detected objects by color (red or blue).
  - Publishes detected objects to the `/detected_objects` topic.
  
- **Key Methods**:
  - `categorize_object`: Categorizes detected objects by their color.
  - `is_target_area`: Filters objects based on their position relative to the camera view.

### 2. ObjectSorter Node
- **Purpose**: Subscribes to detected objects and sorts them based on graspability, proximity, and color.
- **Functionality**:
  - Receives objects from the `/detected_objects` topic.
  - Selects objects based on their accessibility score (proximity, height, and graspability).
  - Uses the `MoveIt!` framework to plan and execute grasping and sorting actions.
  - Moves each object to its corresponding sorting mat based on its color.

- **Key Methods**:
  - `calculate_accessibility`: Prioritizes objects based on their proximity and position.
  - `execute_sorting`: Controls the robot arm to pick and place objects using `MoveGroupCommander` from MoveIt!.

### 3. Object Detection Utilities
- **Purpose**: Provides utility functions for object detection and classification.
- **Functionality**:
  - Contains helper methods for image processing and classification.
  - Methods are used by the `CameraObjectDetection` node to process camera data and detect objects.

- **Key Methods**:
  - `detect_objects`: Identifies and extracts information about objects from the camera feed.

### 4. Launch and Simulation Setup
- **Launch File**: `setup_simulation.launch`
- **Purpose**: Configures and launches the entire sorting system in a simulated environment.
- **Functionality**:
  - Sets up the ROS environment.
  - Launches the robot, camera, and object detection nodes in a simulation.

## Node Structure and Communication

This system utilizes two main ROS nodes:

- **CameraObjectDetection Node**: 
  - Publishes detected objects and their attributes (e.g., color, position).
  - Topic: `/detected_objects`

- **ObjectSorter Node**: 
  - Subscribes to the `/detected_objects` topic.
  - Executes the sorting of objects based on received information.

These nodes communicate via the `/detected_objects` topic, ensuring real-time detection and sorting of objects.

## Object Detection and Categorization

The `CameraObjectDetection` node processes the camera feed to identify and classify objects. Detected objects are categorized by their color using the `categorize_object` function, which determines if an object is red or blue. The node then publishes the objects with relevant information, such as position and color, as custom messages (`DetectedObjects` and `DetectedObject`).

## Sorting Logic

The `ObjectSorter` node prioritizes the sorting of objects based on their graspability, proximity, and height. It calculates an accessibility score to choose which object to sort first. The sorting logic ensures that objects are handled efficiently and transported to the correct sorting mat based on their color.

## Movement and Grasping

The robot arm is controlled using the `MoveGroupCommander` from the MoveIt! framework. The arm moves between predefined positions such as "Sleep" and "Home" to optimize movement. The gripper is used to grasp objects, and feedback mechanisms are in place to confirm successful grasps. The sorting process runs in a continuous loop until all objects are sorted.

## Integration and Execution

The sorting pipeline operates in a loop, with the following sequence of actions:
1. The `CameraObjectDetection` node detects objects and publishes their details.
2. The `ObjectSorter` node receives object details and prioritizes which object to handle based on accessibility.
3. The robot arm moves to grasp the selected object and transports it to the appropriate sorting mat.
4. After sorting, the arm returns to a ready position for the next task.

### ROS Topics:

- **Subscribed Topics**:
    - `/gazebo/locobot/camera/logical_camera_image`: Retrieves the logical camera image containing detected models.
    - `/locobot/joint_states`: Retrieves the current joint states of the robot for verifying the pan and tilt positions.

- **Published Topics**:
    - `/detected_objects`: Publishes detected object information including name, color, position, and bounding box.
    - `/locobot/pan_controller/command`: Sends commands to move the camera pan.
    - `/locobot/tilt_controller/command`: Sends commands to move the camera tilt.


#### How to Run:

1. Build it:
   ```bash
    cd ~/catkin_ws
    catkin_make
    source ~/catkin_ws/devel/setup.bash
   ```
2. Run the simulation launch file:
   ```bash
    roslaunch locobot_sorting setup_simulation.launch
   ```
3. Run the camera_object_detection node:
   ```bash
    rosrun locobot_sorting camera_object_detection.py
    ```
4. 4. Wait for "Ready to take commands for planning group interbotix_arm" for motion planning to function correctly.

5. Run the grasp node:
    ```bash
    rosrun locobot_sorting object_sorter_grasp.py
    ```

## Future Improvements

- **Multi-object Sorting**: Currently, objects are sorted one by one. In future implementations, simultaneous sorting of multiple objects could increase efficiency.
- **3D Object Detection**: Enhancing object detection to work in 3D space, including depth data from the camera, could improve sorting accuracy.
- **Dynamic Color Expansion**: Supporting more colors beyond red and blue for a more diverse sorting application.

