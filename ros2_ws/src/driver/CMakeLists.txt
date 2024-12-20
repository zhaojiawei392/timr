cmake_minimum_required(VERSION 3.8)
project(driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
if (NOT Boost_FOUND)
    message(FATAL_ERROR "Boost not found. Install Boost and ensure the system component is available.")
endif()


add_executable(driver_node src/driver-node.cpp)
target_include_directories(driver_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/coppeliasim/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../kinematics/external/json/include
)

ament_target_dependencies(driver_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

target_link_libraries(driver_node 
    ${CMAKE_CURRENT_SOURCE_DIR}/external/coppeliasim/share/libremoteApiCoppeliasim.so
    Boost::system
)

install(TARGETS
  driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()