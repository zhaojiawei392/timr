cmake_minimum_required(VERSION 3.8)
project(conscious)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(conscious_node src/conscious-main.cpp)
target_include_directories(conscious_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(conscious_node
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  conscious_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()