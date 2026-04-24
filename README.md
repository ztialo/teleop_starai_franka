# teleop_starai_franka

ROS 2 package for **visualizing a leader arm** in RViz and **teleoperating a Franka arm** by mapping the leader end-effector (EEF) motion into Franka base coordinates.

## Overview

- A node to publish/visualize the **leader arm joint states / pose** in RViz.
- A TF-based publisher that outputs the **leader EEF pose relative to the leader base**.
- A controller node (`franky_controller`) that subscribes to **EEF pose commands** + **gripper commands** and sends **absolute commands** to the Franka arm.

## Installation

```bash
# example workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:ztialo/teleop_starai_franka.git
git clone git@github.com:ztialo/lerobot_teleoperator_violin.git

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
Follow the [tutorial](https://wiki.seeedstudio.com/lerobot_starai_arm/) on setting up the STARAI teleoperator and LeRobot environment.

## Usage
1. Connect to leader STARAI arm, visualize in RVIZ, and publish end effector pose with transformation matrix
```
   ros2 launch teleop_starai_franka leader_rviz.launch.py
```
2. Subscribe to end effector pose command ROS2 topic and control Franka arm
```
  python3 scripts/franky_controller.py
```

## CoinFT
### Updates:
Fixed CoinFT frame definition. Created fingertip xforms offset from finger xform, CoinFT sensor xform shares the same origin as the fingertip xform. Then fixed joints are created between finger tip and CoinFT sensor. 

### TODO:
Left and right CoinFT values are opposite, so be sure to run the test again and change the sign of the negative sensor value.

### Usage:
1. First launch Isaac Sim
2. Open test scene from `usd/franka_ft_scene.usd`
3. In Isaac Sim, click on Windows -> script editor -> file -> open: `scripts/keyboard_control.py`
4. Start Simulation
5. Run script
  
