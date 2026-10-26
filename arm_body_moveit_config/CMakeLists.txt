cmake_minimum_required(VERSION 3.8)
project(arm_body_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/add_ground_plane.py
  scripts/check_sim_controllers.py
  scripts/check_gazebo_execution.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
