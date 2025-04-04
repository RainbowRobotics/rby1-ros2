# rby1_ros2

⚠️ **IMPORTANT WARNING**  
**This software is currently under active development, and its stability is not guaranteed.  
Various bugs may occur during use, so please proceed with caution.**

## Installation
- OS: Ubuntu 22.04  
- ROS 2: Humble
### Prerequisites
- install [ROS 2 HUMBLE](https://docs.ros.org/en/humble/Installation.html)
- install [rby1-sdk](https://github.com/RainbowRobotics/rby1-sdk)
    ```shell
    sudo apt install -y build-essential cmake git

    mkdir -p ~/sdk && cd ~/sdk

    #install conan
    pip install conan

    #clone the repository
    git clone --recurse-submodules git@github.com:RainbowRobotics/rby1-sdk.git

    #install or build dependencies
    cd rby1-sdk
    conan install . -s build_type=Release -b missing -of build

    cmake --preset conan-release
    cmake --build --preset conan-release
    ```

- install ROS 2 package dependencies
    ```shell
    sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-urdf \
    ros-humble-urdf-launch \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-transmission-interface \
    ros-humble-moveit \
    ros-humble-moveit-setup-assistant \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-visual-tools \
    ros-humble-rosidl-default-generators \
    ros-humble-action-msgs \
    ros-humble-rclcpp-action \
    ros-humble-rqt \
    ros-humble-rqt-controller-manager
    ```

- set up environment
    ```shell
    export CMAKE_PREFIX_PATH=/opt/ros/humble:$CMAKE_PREFIX_PATH
    export RBY1_SDK_PATH=~/sdk/rby1-sdk
    source /opt/ros/humble/setup.bash
    source ~/rby1_ros2_ws/install/setup.bash
    ```

### Build from source
- create ROS 2 workspace
    ```shell
    mkdir -p ~/rby1_ros2_ws/src
    ```
- clone repo and build `rby1_ros2` packages:
    ```shell
    cd ~/rby1_ros2_ws

    git clone https://github.com/dongridong/rby1-ros2.git src

    colcon build --symlink-install --cmake-args
    source install/setup.bash
    ```  
<br>

# How to use

## manipulation
### bring up
```shell
ros2 launch rby1_bringup rby1_bringup.launch robot_ip:=<Rpc_ip_address:port_number>

# We recommend verifying it in simulation before running on the real robot.
```

### moveit config
```shell
ros2 launch rby1_moveit_config rby1_moveit.launch
```

## Mobile controller
⚠️ **Developer Note**  
Originally, the mobile control feature was intended to allow real-time parameter adjustments via the UI — for example, changing linear and angular velocity on the fly.  

**However, this functionality is not yet implemented.**

Currently, to control the mobile platform, you must manually modify the velocity parameters in the mobile_publisher code, then restart the node to apply the changes.

Please note that this feature is still under development and may contain bugs.  

**As it may cause collisions with surrounding objects or people, please use it with caution.**  


### publish mobile control msg
```shell
ros2 run rby1_mobile_control mobile_publisher
```

### subscribe mobile control msg
```shell
ros2 run rby1_subscriber_pkg 07_mobile_control
```

