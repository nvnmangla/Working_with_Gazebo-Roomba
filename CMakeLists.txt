cmake_minimum_required(VERSION 3.8)
project(roomba)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlebot3_gazebo REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# include directories
include_directories(include)

# making executable
add_executable(walk src/roomba.cpp)

#target dependencies
ament_target_dependencies(walk rclcpp std_msgs tf2 tf2_ros turtlebot3_gazebo)

install(TARGETS
  walk
  DESTINATION lib/${PROJECT_NAME})

ament_package()
