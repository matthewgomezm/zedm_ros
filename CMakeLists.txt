cmake_minimum_required(VERSION 3.8)
project(rover-zed)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(point_cloud_transport REQUIRED) 
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# 2. Define the Executable
add_executable(ORL_node src/main.cpp)

# 3. Link Dependencies to the Executable
ament_target_dependencies(ORL_node
  rclcpp
  sensor_msgs
  point_cloud_transport
  cv_bridge
  image_transport
)

# 4. Install Rules
install(TARGETS ORL_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()