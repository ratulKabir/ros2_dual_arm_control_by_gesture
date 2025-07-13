# ros2_dual_arm_control_by_gesture
Real-time teleoperation of dual 6-DOF robot arms using 2D hand gestures via a single RGB camera in ROS2 Humble.

## Installation

1. **Set up ROS2 and MoveIt2 using MoveIt2 Docker**  
    Follow the guide in [[1]](https://moveit.picknik.ai/humble/doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu.html).

2. **Clone this repository**
    ```bash
    git clone git@github.com:ratulKabir/ros2_dual_arm_control_by_gesture.git
    ```

3. **Install Mediapipe [[2]](https://pypi.org/project/mediapipe/)**
    ```bash
    pip install mediapipe
    ```

4. **Clone pymoveit2 inside your `src` folder**
    ```bash
    git clone https://github.com/AndrejOrsula/pymoveit2.git src/pymoveit2
    ```
    See [[3]](https://github.com/AndrejOrsula/pymoveit2).

5. **Configure Docker for webcam support in VS Code**
    Use the provided [`devcontainer.json`](https://github.com/ratulKabir/ros2_dual_arm_control_by_gesture/blob/main/.devcontainer/devcontainer.json) and adjust your webcam device name if necessary.



## Build

To build the workspace, run:

    ```bash
    colcon build --symlink-install
    ```

## Running the Project

1. **Track hand poses and grip status using camera** <br>
        ```bash
        ros2 run hand_gesture_capture hand_publisher_node
        ```
   <br>**View hand tracking and gripper status results** <br>
        ![Hand Tracking Example](media/hand_tracking_1.gif)

2. **Define your robot in xacro or urdf format in the `src/control_robot/urdf` folder.**

3. **Generate MoveIt2 configuration package [[4]](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html)** <br>
        ```bash
        ros2 launch moveit_setup_assistant setup_assistant.launch.py
        ```

4. **Post MoveIt2 Config Generation Changes** <br>
        
    - In `joint_limits.yaml`, set `max_velocity` to a float (e.g., `1.0` instead of `1`).
    - *(Optional)* In `ros2_control.xacro`, change `load` to `xacro.load`.
    - In `moveit_controllers.yaml`, add the following to all controllers: <br>
            ```yaml
            action_ns: follow_joint_trajectory
            default: true
            ```
    <br>See [[5]](https://github.com/moveit/moveit2/issues/1506#issuecomment-1215014684).

5. **Use MoveIt2 interface to move the robot.**

> **Note:** Using one node with two MoveIt2 objects may lock the node and eventually both nodes. Therefore, I am using two different nodes. To run the nodes, do the following: <br>
`ros2 run control_robot move_to_pose`
`ros2 run control_robot move_to_pose_r1`

## Running the Full Stack

To run the entire system, use two terminals (Source the workspace in each):

**Terminal 1:** Launch the MoveIt2 configuration package:
```bash
ros2 launch moveit2_dual_arm demo.launch.py
```

**Terminal 2:** Start the hand tracking node and both robot control nodes:
```bash
ros2 launch control_robot full_stack.launch.py
```

<br>**You'll see results like the gif below. Here, I am moving the robots updwards using my hands.** <br>
        ![Hand Tracking Example](media/full.gif)


## REFERENCES

[1] [How to Set Up MoveIt 2 Docker Containers in Ubuntu](https://moveit.picknik.ai/humble/doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu.html) <br>
[2] [Mediapipe library](https://pypi.org/project/mediapipe/) <br>
[3] [Pymoveit2 library](https://github.com/AndrejOrsula/pymoveit2) <br>
[4] [MoveIt Setup Assistant](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html) <br>
[5] [Moveit2 issue #1506](https://github.com/moveit/moveit2/issues/1506#issuecomment-1215014684) <br>
[6] [DexPilot: Vision Based Teleoperation of Dexterous Robotic Hand-Arm System](https://arxiv.org/pdf/1910.03135) <br>
[7] [Motor-Imagery-Based Teleoperation of a Dual-Arm Robot Performing Manipulation Tasks](https://ieeexplore.ieee.org/abstract/document/8486736) <br>
[8] [Differentiable Integrated Motion Prediction and Planning With Learnable Cost Function for Autonomous Driving](https://ieeexplore.ieee.org/document/10154577)



## Other Resources

- **Video:** [6 DoF robot arm creation from scratch and visualization in RVIZ](https://www.youtube.com/watch?v=kh2yhsKZRQ8&ab_channel=LearnroboticswithROS)
- **Video:** [Manipulate robot using MoveIt and visualization in RVIZ](https://www.youtube.com/watch?v=18mouzWyqRo&ab_channel=Softillusion)
- **Repo:** [Hand Movement Tracking using MediaPipe and OpenCV](https://github.com/noorkhokhar99/Hand-Movement-Tracking-)
- **Video:** [MoveIt Configuration Package](https://www.youtube.com/watch?v=5dAHoK9BbgE&ab_channel=LearnroboticswithROS)
- **Video:** [MoveIt2 Humble Installation](https://www.youtube.com/watch?v=c6Bxbq8UdaI&ab_channel=AleksandarHaberPhD)



## Improvements & Future Work

For possible features, and future improvements, please refer to [IMPROVEMENTS.md](./IMPROVEMENTS.md).