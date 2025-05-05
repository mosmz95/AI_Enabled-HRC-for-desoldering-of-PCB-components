cmake_minimum_required(VERSION 3.8)
project(ur5e_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Optional: Disables compiler-specific extensions

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ur_rtde REQUIRED)

find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(custom_interfaces REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit
  moveit_msgs
  moveit_visual_tools
  moveit_ros_planning_interface)

include_directories(include)

# add_executable(rtde_usage src/rtde_test.cpp)
# target_link_libraries(rtde_usage  PRIVATE ur_rtde::rtde)

add_executable(obj_test src/test_obj.cpp src/data_object.cpp)

add_executable(sequence_logic src/move_robot_logicv1.cpp src/moverobotclass.cpp src/data_object.cpp src/heatsequencelogic.cpp)
ament_target_dependencies(sequence_logic rclcpp Boost custom_interfaces  ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(automatic_desoldering src/move_robot_automatic_sequence.cpp src/moverobotclass.cpp src/data_object.cpp src/desoldering_automatic_sequence.cpp)
ament_target_dependencies(automatic_desoldering rclcpp Boost custom_interfaces  ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS
  # rtde_usage
  automatic_desoldering
  obj_test
  sequence_logic
  DESTINATION   lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
