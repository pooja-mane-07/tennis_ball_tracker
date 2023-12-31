cmake_minimum_required(VERSION 3.8)
project(tennis_ball_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

add_executable(tennis_ball_publisher 
  src/tennis_ball_publisher.cpp
)

ament_target_dependencies(tennis_ball_publisher rclcpp OpenCV sensor_msgs)

add_executable(tennis_ball_listener 
  src/tennis_ball_listener.cpp
)

ament_target_dependencies(tennis_ball_listener rclcpp OpenCV sensor_msgs)

install(TARGETS
tennis_ball_publisher
tennis_ball_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  resources 
  launch
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
