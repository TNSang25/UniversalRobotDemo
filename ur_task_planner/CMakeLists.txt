cmake_minimum_required(VERSION 3.8)
project(ur_task_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteSkill.srv"
  "msg/ObservedScene.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(skill_executor_node src/skill_executor_node.cpp)
target_compile_features(skill_executor_node PUBLIC cxx_std_17)
ament_target_dependencies(skill_executor_node
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  control_msgs
  rclcpp_action
  moveit_msgs
  shape_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(skill_executor_node "${cpp_typesupport_target}")

install(TARGETS
  skill_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scene_observer_node.py
  scripts/llm_planner_node.py
  scripts/check_install.py
  DESTINATION lib/${PROJECT_NAME}
)

# Imported by llm_planner_node.py, has to be installed next to it
install(FILES
  scripts/plan_validator.py
  scripts/scene_geometry.py
  scripts/cube_perception.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_llm_occupancy test/test_llm_occupancy.py)
  ament_add_pytest_test(test_scene_observation test/test_scene_observation.py)
  ament_add_pytest_test(test_plan_validator test/test_plan_validator.py)
  ament_add_pytest_test(test_installation test/test_installation.py)

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
