cmake_minimum_required(VERSION 3.8)
project(opencv_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)
# find_package(OpenCV REQUIRED)

add_executable(example_opencv_node src/example_opencv_node.cpp)
ament_target_dependencies(example_opencv_node rclcpp std_msgs sensor_msgs cv_bridge image_transport ament_index_cpp OpenCV)

add_executable(orb_example src/orb_example.cpp)
ament_target_dependencies(orb_example rclcpp std_msgs sensor_msgs cv_bridge image_transport ament_index_cpp OpenCV)

install(TARGETS
   example_opencv_node
   orb_example
   DESTINATION lib/${PROJECT_NAME}
 )

# Install images so they are available in share directory
install(DIRECTORY images/
  DESTINATION share/${PROJECT_NAME}/images
  FILES_MATCHING PATTERN "*.jpg" PATTERN "*.png" PATTERN "*.jpeg"
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
