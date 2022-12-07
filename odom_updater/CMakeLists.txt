cmake_minimum_required(VERSION 3.8)
project(odom_updater)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)



add_executable(odom_updater src/broadcast_pose.cpp)
ament_target_dependencies(odom_updater tf2_ros nav_msgs rclcpp geometry_msgs)



#############
## Install ##
#############
install(TARGETS odom_updater DESTINATION lib/${PROJECT_NAME}/)



ament_package()
