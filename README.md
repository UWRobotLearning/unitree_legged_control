# unitree_legged_control

# Configuration

 ```bash
export UNITREE_SDK_VERSION=3_3_3
export UNITREE_LEGGED_SDK_PATH=~/catkin_ws/src/unitree_legged_control/unitree_legged_sdk
export UNITREE_PLATFORM="arm64"
```
# Problems
- Since the A1_sport version is less than 1.20, we reverted back to 3.2 sdk version for deploying high level commands.
- While we are able to send high level commands using this sdk, we are not able receive low level motor states dring high level mode.


# Possible Solutions
- If we can upgrade the A1_sports version from 1.16 to 1.20 or above, then it resolves everything.

# Problems resolved :
- A1_sport version less than 1.20.
- 3.2 sdk incompatible for sending and receiving high level commands and low level commands respectively.

# SDKs

| SDK           | Low mode      | High mode     |
| ------------- | ------------- | ------------- |
| 3.2   |  ✅   |   ✅   |
| 3.34  |  ✅   |   ✅  |
.
.
| 3.8   |   ✅  |   ❌   |



# TBD
- Bash script for the installation of prerequisites.
- Rosbag collector launch file.
- Parser for rostopics from ros bag
- And a proper readme 😅