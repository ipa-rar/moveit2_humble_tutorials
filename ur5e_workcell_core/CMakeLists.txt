cmake_minimum_required(VERSION 3.5)
project(ur5e_workcell_core)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fake_ar_publisher REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Interface generation dependencies
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# moveit related
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LocalizePart.srv"
  DEPENDENCIES geometry_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(vision_node src/vision_node.cpp)
ament_target_dependencies(vision_node fake_ar_publisher tf2_ros tf2_geometry_msgs rclcpp)
target_link_libraries(vision_node "${cpp_typesupport_target}") 

add_executable(scannplan_node src/scan_n_plan.cpp)
ament_target_dependencies(scannplan_node moveit_msgs moveit_ros_planning_interface rclcpp)
target_link_libraries(scannplan_node "${cpp_typesupport_target}") 

install(TARGETS vision_node scannplan_node
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
