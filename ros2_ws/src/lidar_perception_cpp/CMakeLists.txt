cmake_minimum_required(VERSION 3.8)
project(lidar_perception_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 core and other packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)


# External libraries


# Adding executable
add_executable(lidar_perception_node src/lidar_perception.cpp)


target_link_libraries(lidar_perception_node
  ${PCL_LIBRARIES}
  ${pcl_conversions_LIBRARIES} 
)


ament_target_dependencies(lidar_perception_node rclcpp sensor_msgs tf2_ros)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS lidar_perception_node 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
