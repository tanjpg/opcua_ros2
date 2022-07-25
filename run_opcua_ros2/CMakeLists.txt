cmake_minimum_required(VERSION 3.5)
project(run_opcua_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread program_options)
find_package(opcua_ros2 REQUIRED)

add_executable(demo_client src/opcua_ros2_client.cpp)
add_executable(demo_server src/opcua_ros2_server.cpp)

target_link_libraries(demo_client
  ${FreeOpcUa_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(demo_server
  ${FreeOpcUa_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(demo_client
  opcua_ros2
  rclcpp
  FreeOpcUa)

ament_target_dependencies(demo_server
  opcua_ros2
  rclcpp
  FreeOpcUa)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  demo_client
  demo_server
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
