
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---
# Working_with_Gazebo-Roomba

### Overview

This is a ROS Package that defines a basic obstacle_avoidance using Gazebo and Turtlebot3.

### Dependencies/ Assumptions
- OS : Ubuntu 20.04 
- ROS2 Distro : ROS2 Galactic
- Package build type : ```ament_cmake ```
- Package dependencies : ```rclcpp```, ```std_msgs``` ,```turtlebot3_gazebo```
- ROS2 Galactic Installation : [link](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

## How to Run the ROS Package
### Build Instructions
```
cd <your_ROS2_ws>/src
git clone https://github.com/nvnmangla/Working_with_Gazebo-Roomba.git
cd ..   
rosdep install -i --from-path src --rosdistro galactic -y
colcon build --packages-select roomba
source . install/setup.bash
source ~/<your ROS2 installation>/opt/ros/galactic/setup.bash
```

### Run Publisher
To run the publisher node, open a new terminal and run:
```
cd <your_ROS2_ws>
. install/setup.bash
ros2 run roomba walk
```
Make sure you have  installed turtlebot3 and you are not i conda enviornment 

### Launch File
To launch, Run the following commands:
```
cd <ROS2_ws>/src/roomba/launch
```
If you want to launch with bag recording on
```
ros2 launch launch_roomba.yaml ros_record:=true 
```
```
ros2 launch launch_roomba.yaml ros_record:=false 
```
otherwise
### ROS2 Bags
Bag files are available in
```
<ros_ws>/src/roomba/results/bag
```
## Results
The results after running the following commands are stored in the <your_package>/results folder.

### rqt Console
```
 ros2 run rqt_console rqt_console

```
### cppcheck
Run the following command from the root directory of your ROS package
```
cppcheck --enable=all --std=c++17 ./src/*.cpp ./include/*.hpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
### cpplint
Run the following command from the root directory of your ROS package
```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp ./include/*.hpp > ./results/cpplint.txt
```
### Google Styling format
This command will autstyle the files.
```
clang-format -style=Google -i src/*.cpp include/*.hpp
```