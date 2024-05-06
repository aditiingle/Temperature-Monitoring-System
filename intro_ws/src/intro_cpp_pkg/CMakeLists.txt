cmake_minimum_required(VERSION 3.8)
project(intro_cpp_pkg)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp example_interfaces)

install (TARGETS 
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
