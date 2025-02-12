cmake_minimum_required(VERSION 3.8)
project(controller_pub_kumo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(
  DIRECTORY   launch scripts
  DESTINATION share/${PROJECT_NAME}
)
install(
    PROGRAMS
    scripts/robot_control.py
    scripts/robot_control_sim.py
    scripts/Move.py
    scripts/kalman_filter.py
    scripts/kalman_robot_control.py
    launch/control_kumo.launch.py
    launch/kalman_control_kumo.launch.py
    DESTINATION lib/${PROJECT_NAME}
  )

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
