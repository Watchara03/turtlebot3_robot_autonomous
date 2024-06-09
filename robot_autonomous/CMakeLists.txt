cmake_minimum_required(VERSION 3.8)
project(robot_autonomous)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(example_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
# New Eidtions
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(pub_occupancy_grid src/pub_occupancy_grid.cpp)
ament_target_dependencies(pub_occupancy_grid rclcpp nav_msgs)

# add_executable(ex_service src/add_two_ints_server.cpp)
# ament_target_dependencies(ex_service rclcpp example_interfaces)

# # add_executable(ex_client src/add_two_ints_client.cpp)
# # ament_target_dependencies(ex_client rclcpp example_interfaces)

install(TARGETS
  pub_occupancy_grid
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/single_goal_nav.py 
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/multi_waypoint.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/initial_pose.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/Test_goal.py 
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/int_pose.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/gazebo_multi_waypoint.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/arrival_server.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/gazebo_multi_waypoint.py
  DESTINATION lib/${PROJECT_NAME})


# install(PROGRAMS
#   scripts/test_service.py
#   DESTINATION lib/${PROJECT_NAME})



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
