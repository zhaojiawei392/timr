cmake_minimum_required(VERSION 3.8)
project(kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(qpOASES REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(kinematics_node src/kinematics-node.cpp)
target_include_directories(kinematics_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(kinematics_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

target_link_libraries(kinematics_node 
  qpOASES::qpOASES 
  nlohmann_json
)

install(TARGETS
  kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()