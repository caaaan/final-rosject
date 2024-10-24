# locobot_sorting Package

The `locobot_sorting` package is responsible for detecting, categorizing, and later sorting objects based on color and type using the LoCoBot's camera and arm. The current implementation includes object detection using a logical camera and controlling the pan and tilt of the robot's camera.

## Current Functionality

## camera_object_detection.py

### Camera Object Detection Node

The `camera_object_detection` node detects objects using the robot's logical camera, categorizes them by type and color, and publishes detected object information. The node also moves the robot's camera using pan and tilt to scan the environment for objects.

#### Key Features:

- **Object Detection**: 
    - The script subscribes to `/gazebo/locobot/camera/logical_camera_image` to detect objects in the environment. Detected objects are categorized into `red`, `blue`, or `unknown` based on their type.
    - The node filters out irrelevant objects like the ground plane and target areas.
  
- **Camera Control**: 
    - The node controls the camera's pan and tilt using ROS topics `/locobot/pan_controller/command` and `/locobot/tilt_controller/command`.
    - The camera scans specific positions to cover a large area and maximize the chance of detecting objects.

- **Categorization**: 
    - Objects are categorized based on a predefined list of known red and blue objects (e.g., `red_big_ball`, `blue_cylinder`). Unknown objects are labeled accordingly.

- **Bounding Box Calculation**: 
    - The bounding box for each detected object is calculated based on its position (`min` and `max` points) and is published along with the object information.

- **ROS Integration**: 
    - Detected objects are published on the `/detected_objects` topic using custom messages (`DetectedObject`, `DetectedObjects`), which include the object's name, label (color), position, bounding box, and probability.

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
### ROS Topics:

- **Subscribed Topics**:
    - `/gazebo/locobot/camera/logical_camera_image`: Retrieves the logical camera image containing detected models.
    - `/locobot/joint_states`: Retrieves the current joint states of the robot for verifying the pan and tilt positions.

- **Published Topics**:
    - `/detected_objects`: Publishes detected object information including name, color, position, and bounding box.
    - `/locobot/pan_controller/command`: Sends commands to move the camera pan.
    - `/locobot/tilt_controller/command`: Sends commands to move the camera tilt.
