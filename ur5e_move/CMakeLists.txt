cmake_minimum_required(VERSION 3.8)
project(ur5e_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ur_rtde REQUIRED)




include_directories(include)

add_executable(rtde_usage src/rtde_test.cpp)
target_link_libraries(rtde_usage  PRIVATE ur_rtde::rtde)

install(TARGETS
  rtde_usage
  DESTINATION   lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
