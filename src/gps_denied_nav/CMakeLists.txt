cmake_minimum_required(VERSION 3.8)
project(gps_denied_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Suppress warnings from MAVLink headers about packed member addresses
  add_compile_options(-Wno-address-of-packed-member)
endif()

set(OpenCV_DIR "/home/ubuntu/opencv/build")

# Set RPATH so OpenCV libraries are found at runtime
set(CMAKE_INSTALL_RPATH "/home/ubuntu/opencv/build/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(mavlink REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(main 
  src/main.cpp
  src/follow_path_node.cpp
  src/feature_processor.cpp)
ament_target_dependencies(main rclcpp std_msgs sensor_msgs cv_bridge image_transport ament_index_cpp mavros_msgs geometry_msgs visualization_msgs)
target_link_libraries(main ${OpenCV_LIBS})

add_executable(create_path src/create_path.cpp)
ament_target_dependencies(create_path rclcpp std_msgs sensor_msgs cv_bridge image_transport ament_index_cpp mavros_msgs geometry_msgs rosbag2_cpp rosbag2_storage rosbag2_transport)
target_link_libraries(create_path ${OpenCV_LIBS})

# drone_control is commented out - source files moved to drone_control_(unused)
# add_executable(drone_control 
#   src/drone_control/drone_controller_node.cpp
#   src/drone_control/drone_mav_connection.cpp)
# ament_target_dependencies(drone_control rclcpp ament_index_cpp mavlink mavros_msgs geometry_msgs std_msgs std_srvs)

install(TARGETS
  main
  create_path
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../python_scripts/gimbal_leveler.py
  DESTINATION lib/${PROJECT_NAME}
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

# Install launch and config directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
