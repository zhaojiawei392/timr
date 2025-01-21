cmake_minimum_required(VERSION 3.8)
project(driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(yaml-cpp REQUIRED)
if (NOT Boost_FOUND)
    message(FATAL_ERROR "Boost not found. Install Boost and ensure the system component is available.")
endif()


add_executable(driver_node src/driver-node.cpp)
target_include_directories(driver_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/coppeliasim/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/json/include
)

ament_target_dependencies(driver_node
  rclcpp
  sensor_msgs
)

target_link_libraries(driver_node 
    ${CMAKE_CURRENT_SOURCE_DIR}/external/coppeliasim/share/libremoteApiCoppeliasim.so
    Boost::system
)

add_executable(my_test src/test.cpp)
target_include_directories(my_test PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/coppeliasim/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/yaml/include
)

ament_target_dependencies(my_test
  rclcpp
) 

target_link_libraries(my_test
  Boost::system
  yaml-cpp::yaml-cpp
)

install(TARGETS
  driver_node
  my_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()