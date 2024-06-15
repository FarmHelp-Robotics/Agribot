# agribot_src
Collection of packages for agri-robotics. Developed and testing with ROS1 Melodic.

## Installation
```bash
mkdir -p agribot_ws/src && cd agribot_ws # Skip this if you already have a workspace
# from root of the workspace

git clone https://github.com/FarmHelp-Robotics/agribot_src src/agribot_src
rosdep install --from-paths src --ignore-src -r -y

# enjoy! :D
```
