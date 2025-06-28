# kinova_robot
This repository is used to control a kinova robotic arm in both real-life and simulation

## Building the ROS2 workspace
1. Make sure that `colcon`, its extensions, and `vcs` are installed:
   ```
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```

2. Create a new ROS2 workspace:
   ```
   export COLCON_WS=~/workspace/ros2_kortex_ws
   mkdir -p $COLCON_WS/src
   ``

3. Pull relevant packages:
   ```
   cd $COLCON_WS
   git clone https://github.com/abed-almrad/kinova_robot.git
   vcs import src --skip-existing --input src/kinova_robot/rl_dependencies.repos
   vcs import src --skip-existing --input src/kinova_robot/simulation_dependencies.repos
   ```

4. Install dependencies, compile:
   ```
   rosdep install --ignore-src --from-paths src -y -r
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
   By default, colcon will use as much resources as possible to build the ROS2 workspace. This can temporarily freeze or even crash your machine. You can limit the number of threads used to avoid this issue, I found a good tradeoff between build time and resource utilisation by setting it to 3 :
   ```
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
   ```

5. source the workspace:
   ``` 
   source ~/workspace/ros2_kortex_ws/install/setup.bash
   ```

## Real-Life Robot Control

0. Start the previously created `gen3_rl_control.launch.py` file to bringup and visualize the Gen3 7DoF robotic arm from Kinova:

```
ros2 launch kortex_bringup gen3_rl_control.launch.py robot_ip:=192.168.1.10 launch_rviz:=true
```

1. You can command the arm by publishing Joint Trajectory messages directly to the joint trajectory controller with joint positions are in **radians**:

Candle configuration:

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
  points: [
    { positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 10 } },
  ]
}" -1
```

Non-singularity configuration:

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
  points: [
    { positions: [0.0098, -0.28, 0, 2, 0.1309, 1.0416, -1.6], time_from_start: { sec: 10 } },
  ]
}" -1
```

2. You can also command the arm using Twist messages. Before doing so, you must active the `twist_controller` and deactivate the `joint_trajectory_controller`:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [twist_controller],
  deactivate_controllers: [joint_trajectory_controller],
  strictness: 1,
  activate_asap: true,
}"
```
Once the `twist_controller` is activated, You can publish Twist messages on the `/twist_controller/commands` topic to command the arm:

Move the end effector down

```
ros2 topic pub /twist_controller/commands geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.01}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```
Move the end effector up
```
ros2 topic pub /twist_controller/commands geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: -0.01}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

Move the end effector sideways:
```
ros2 topic pub /twist_controller/commands geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.01, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

```
ros2 topic pub /twist_controller/commands geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -0.01, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

Rotate around x-axis:
```
ros2 topic pub /twist_controller/commands geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: -2, y: 0.0, z: 0.0}}" -1
```

```
ros2 topic pub /twist_controller/commands geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 2, y: 0.0, z: 0.0}}" -1
```

Rotate around y-axis:
```
ros2 topic pub /twist_controller/commands geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 2.0, z: 0.0}}" -1
```

```
ros2 topic pub /twist_controller/commands geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: -2.0, z: 0.0}}" -1
```

Stop the robot:
```
ros2 topic pub /twist_controller/commands geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

3. If you wish to use the `joint_trajectory_controller` again to command the arm using JointTrajectory messages, run the following:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [joint_trajectory_controller],
  deactivate_controllers: [twist_controller],
  strictness: 1,
  activate_asap: true,
}"
```

Then command the robot:

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
  points: [
    { positions: [0.0098, -0.28, 0, 2, 0.1309, 1.0416, -1.6], time_from_start: { sec: 10 } },
  ]
}" -1
```

4. You can move the gripper by calling the Action server with the following command and setting the desired `position` of the gripper (`0.0=open`, `1.0=close`):

Open the gripper:

```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.0, max_effort: 100.0}}"
```

Close the gripper:

```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 1.0, max_effort: 100.0}}"
```
5. Listen to the joint states:
```
ros2 topic echo /joint_states
```

6. On the topic `/fault_controller/internal_fault` of type example_interfaces::msg::Bool the information if robot is currently faulted can be found:

Listen to the `/fault_controller/internal_fault`:
```
ros2 topic echo /fault_controller/internal_fault
```

Trigger a robot fault the call the service for resetting the fault (example_interfaces::srv::Trigger).

```
ros2 service call /fault_controller/reset_fault example_interfaces/srv/Trigger
```

7. Controllers activation/deactivation:

Activate controllers to enable robot control.

```
ros2 control switch_controllers --activate joint_trajectory_controller robotiq_gripper_controller
```

Deactivate all the controllers using robot.

```
ros2 control switch_controllers --deactivate joint_trajectory_controller robotiq_gripper_controller
```

## Simulated Robot Control

0. Start the previously created `gen3_sim_control.launch.py` file to bringup the simulation of the Gen3 7DoF robotic arm from Kinova:

```
ros2 launch kortex_bringup gen3_sim_control.launch.py
```

1. You can command the arm by publishing Joint Trajectory messages directly to the joint trajectory controller with joint positions are in **radians**

Non-singularity configuration:

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
  points: [
    { positions: [0.0098, -0.28, 0, 2, 0.1309, 1.0416, -1.6], time_from_start: { sec: 10 } },
  ]
}" -1
```

2. You can move the gripper by calling the Action server with the following command and setting the desired `position` of the gripper (`0.1=open`, `0.7=close`):

Open the gripper:

```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.1, max_effort: 100.0}}"
```

Close the gripper:

```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.7, max_effort: 100.0}}"
```

3. Controllers activation/deactivation:

Deactivate all the controllers using robot.

```
ros2 control switch_controllers --deactivate joint_trajectory_controller robotiq_gripper_controller
```

Activate controllers to enable robot control.

```
ros2 control switch_controllers --activate joint_trajectory_controller robotiq_gripper_controller
```

4. The twist and the fault controllers were not developed to be used for simulations

