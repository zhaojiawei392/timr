cmake_minimum_required(VERSION 3.8)
project(driver)

option(BUILD_TESTING "Build tests" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(timr_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_executable(driver_node src/driver_node.cpp)
target_include_directories(driver_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(driver_node
  rclcpp
  std_msgs
  sensor_msgs
  yaml-cpp
  timr_msgs
)

target_link_libraries(driver_node
  yaml-cpp::yaml-cpp
)

if (BUILD_TESTING)
  ament_add_gtest(helper test/helper.cpp)
  target_include_directories(helper PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(helper
    rclcpp
  )


  ament_add_gtest(joint_driver test/joint_driver.cpp)
  target_include_directories(joint_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(joint_driver
    rclcpp
  ) 
  target_link_libraries(joint_driver
    yaml-cpp::yaml-cpp
  )


  ament_add_gtest(robot_driver test/robot_driver.cpp)
  target_include_directories(robot_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(robot_driver
    rclcpp
  ) 
  target_link_libraries(robot_driver
    yaml-cpp::yaml-cpp
  )
endif()

install(TARGETS driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()