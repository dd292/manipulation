cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
project(dual_franka_model)

# Required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)


# Include MuJoCo
include_directories(/usr/local/include/mujoco)
link_directories(/usr/local/lib)

#Include Eigen
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(mujoco_node src/mujoco_node.cpp)
ament_target_dependencies(mujoco_node rclcpp sensor_msgs ament_index_cpp Eigen3)
target_link_libraries(mujoco_node mujoco glfw GL m)


install(TARGETS
  mujoco_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch mjcf
  DESTINATION share/${PROJECT_NAME}
)
ament_package()