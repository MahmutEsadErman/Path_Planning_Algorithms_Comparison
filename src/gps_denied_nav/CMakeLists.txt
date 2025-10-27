cmake_minimum_required(VERSION 3.8)
project(gps_denied_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")

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
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(save_video src/save_video.cpp)
ament_target_dependencies(save_video rclcpp std_msgs sensor_msgs cv_bridge image_transport ament_index_cpp)
target_link_libraries(save_video ${OpenCV_LIBS})

add_executable(detect_features_orb src/detect_features_orb.cpp)
ament_target_dependencies(detect_features_orb rclcpp std_msgs sensor_msgs cv_bridge image_transport ament_index_cpp)
target_link_libraries(detect_features_orb ${OpenCV_LIBS})

add_executable(detect_features_sift src/detect_features_sift.cpp)
ament_target_dependencies(detect_features_sift rclcpp std_msgs sensor_msgs cv_bridge image_transport ament_index_cpp)
target_link_libraries(detect_features_sift ${OpenCV_LIBS})


# add_executable(detect_features_surf src/detect_features_surf.cpp)
# ament_target_dependencies(detect_features_surf rclcpp std_msgs sensor_msgs cv_bridge image_transport ament_index_cpp)
# target_link_libraries(detect_features_surf ${OpenCV_LIBS})

add_executable(drone_control 
  src/drone_control/drone_control_node.cpp
  src/drone_control/drone_mav_connection.cpp)
ament_target_dependencies(drone_control rclcpp ament_index_cpp mavlink geometry_msgs std_msgs std_srvs)
target_include_directories(drone_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(TARGETS
  save_video
  detect_features_orb
  detect_features_sift
  drone_control
#  detect_features_surf
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
