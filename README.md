# unitree_legged_control

# Configuration

 ```bash
export UNITREE_SDK_VERSION=3_2
export UNITREE_LEGGED_SDK_PATH=~/catkin_ws/src/unitree_legged_control/unitree_legged_sdk
export UNITREE_PLATFORM="arm64"
```
# Notes
- Since the A1_sport version is less than 1.20, we reverted back to 3.2 sdk version for deploying high level commands.
- While we are able to send high level commands using this sdk, the sdk does not send low level motor states via LCM.
- Although the ROS wrapper is functional for this sdk, it won't be able to pulish low level state topics.

# TBD
- Bash script for the installation of prerequisites.
- Rosbag collector launch file.
- Parser for rostopics from ros bag
- And a proper readme 😅