# aubo_ign_gazebo


## Compatibility

| **Supported OS**          | **Supported ROS2 distribution**                         |
|---------------------------|---------------------------------------------------------|
| Ubuntu 22.04              | [Humble](https://docs.ros.org/en/humble/index.html) |

To install Ignition Gazebo (now known as Gazebo Garden) for integrating ROS 2 Humble on Ubuntu 22.04 and using it with Aubo robot models i5 and i10, you'll need to install both ROS 2 Humble and Ignition Gazebo along with additional dependencies to support the Aubo robots. Here’s an installation guide to set up this environment:

## Getting Started

This project was developed for ROS2 Humble on Ubuntu 22.04. Other versions of Ubuntu and ROS2 may work, but are not officially supported.

1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. Install [Ignition Gazebo (Fortress)](https://gazebosim.org/docs/fortress/install_ubuntu/)
3. Install Ros ignition bridge
   Add https://packages.ros.org
   ```bash
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
     curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
     sudo apt-get update
   ```
   Install `ros_gz`
   ```bash
     sudo apt install ros-humble-ros-gz
   ```
4. Install `colcon` and additional ROS packages:

    ```bash
    sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep2 \
    libeigen3-dev \
    ros-humble-ign-ros2-control \
    ros-humble-xacro \
    ros-humble-tinyxml2-vendor \
    ros-humble-ros2-control \
    ros-humble-realtime-tools \
    ros-humble-control-toolbox \
    ros-humble-moveit \
    ros-humble-ros2-controllers \
    ros-humble-test-msgs \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2
    ```

5. Setup workspace:

    ```bash
    mkdir -p ~/aubo_ros2_ws/src
    cd ~/aubo_ros2_ws/src
    git clone -b humble https://github.com/kittawat-thongpud/aubo_ign_gazebo.git
    cd ~/aubo_ros2_ws
    ```

6. Install dependencies:

    ```bash
    cd ~/aubo_ros2_ws
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
    ```

7. Build and source the workspace:

    ```bash
    cd ~/aubo_ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash
    ```

**NOTE**: Remember to source the setup file and the workspace whenever a new terminal is opened:

# Simulation aubo with ignition gazebo + moveit

Change model to aubo_i5 or aubo_i10 to select model to run
```bash
ros2 launch <model> simulation.launch.py
```

## Example Aubo i5

```bash
ros2 launch aubo_i5 simulation.launch.py
```

## Integrate Depth Camera

```bash
ros2 launch aubo_i5_depth simulation.launch.py
```
