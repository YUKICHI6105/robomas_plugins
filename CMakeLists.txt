cmake_minimum_required(VERSION 3.20)
project(robomas_plugins)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(include ${Boost_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Frame.msg"
  "msg/RobomasFrame.msg"
  "msg/RobomasTarget.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)


add_library(robomas_bridge SHARED
  src/robomas_bridge.cpp)

target_link_libraries(robomas_bridge "${cpp_typesupport_target}")


target_link_libraries(robomas_bridge
  rclcpp::rclcpp
  rclcpp_components::component
  std_msgs::std_msgs
)

rclcpp_components_register_nodes(robomas_bridge "robomas_bridge::RobomasBridge")

install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

install(TARGETS
  robomas_bridge
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}  
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()