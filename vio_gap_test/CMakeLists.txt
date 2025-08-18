cmake_minimum_required(VERSION 3.8)
project(vio_gap_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# Add executable
add_executable(vio_gap_simulator src/vio_gap_simulator.cpp)
ament_target_dependencies(vio_gap_simulator 
  rclcpp 
  geometry_msgs 
  nav_msgs
  mavros_msgs
)

# Install targets
install(TARGETS
  vio_gap_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install scripts
install(PROGRAMS
  scripts/analyze_estimator.py
  scripts/mavros_offboard.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
