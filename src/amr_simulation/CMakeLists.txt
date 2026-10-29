cmake_minimum_required(VERSION 3.8)
project(amr_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(gazebo_ros REQUIRED)
#find_package(nav2_bringup REQUIRED)
#find_package(robot_state_publisher REQUIRED)

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
install(DIRECTORY launch
  rviz
  config
  maps
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/human_awareness_node.py
  scripts/fsm.py
  scripts/adaptive_velocity_node.py
  scripts/astar_benchmark.py
  scripts/hitung_jarak_astar.py
  scripts/camera_publisher.py
  scripts/arduino_bridge.py
  scripts/rplidar_bridge.py
  scripts/laser_filter_node.py
  scripts/benchmark_skripsi.py
  scripts/ukur_peta_skripsi.py
  scripts/uji_amcl_skripsi.py
  scripts/benchmark_astar_vs_dijkstra.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
