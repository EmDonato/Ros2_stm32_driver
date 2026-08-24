cmake_minimum_required(VERSION 3.10)
project(stm32_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(stm32_driver_node
  src/stm32_driver_node.cpp
)

target_include_directories(stm32_driver_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/stm32_driver>
    $<INSTALL_INTERFACE:include>
    ${Boost_INCLUDE_DIRS}
)

ament_target_dependencies(stm32_driver_node
  geometry_msgs
  rclcpp
  robot_interfaces
  sensor_msgs
  std_msgs
  tf2
)

install(TARGETS stm32_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
