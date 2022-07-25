cmake_minimum_required(VERSION 3.5)
project(opcua_ros2)

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem thread program_options)
find_package(FreeOpcUa)
find_package(spdlog)
find_package(rclcpp)

set(FreeOpcUa_LIBRARIES opcuacore opcuaclient opcuaserver opcuaprotocol)

include_directories(
  PUBLIC
  include)

add_library(opcua_interface
  SHARED
  src/opcua_ros2_client.cpp
  src/opcua_ros2_server.cpp
)

get_target_property(target_options opcua_interface COMPILE_OPTIONS)
list(REMOVE_ITEM target_options "-Wpedantic")
set_property(TARGET opcua_interface PROPERTY COMPILE_OPTIONS ${target_options})

ament_target_dependencies(
  opcua_interface
  rclcpp
  FreeOpcUa)

target_link_libraries(opcua_interface
  ${FreeOpcUa_LIBRARIES}
  ${Boost_LIBRARIES}
)

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

install(
  TARGETS opcua_interface
  EXPORT export_opcua_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

install(
  DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(
  include
)

ament_export_libraries(opcua_interface)
ament_export_targets(export_opcua_interface HAS_LIBRARY_TARGET)

ament_export_dependencies(
  FreeOpcUa
  Boost
  spdlog
)

ament_package()
