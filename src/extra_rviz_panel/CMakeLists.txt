cmake_minimum_required(VERSION 3.8)
project(extra_rviz_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenCV_DIR "/home/ubuntu/opencv/build")
set(CMAKE_INSTALL_RPATH "${OpenCV_DIR}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(mavros_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/extra_rviz_panel/rosbag2_panel.hpp
  include/extra_rviz_panel/drone_panel.hpp
)

add_library(extra_rviz_panel_lib SHARED
  src/rosbag2_panel.cpp 
  src/drone_panel.cpp
  ${MOC_FILES}
)

target_include_directories(extra_rviz_panel_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(extra_rviz_panel_lib
  pluginlib
  rviz_common
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  sensor_msgs
  geometry_msgs
  tf2
  cv_bridge
  mavros_msgs
)
target_link_libraries(extra_rviz_panel_lib ${OpenCV_LIBS} Qt5::Widgets)
install(TARGETS extra_rviz_panel_lib
        EXPORT export_rviz_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

# Path Visualizer Node
add_executable(path_visualizer src/path_visualizer.cpp)
ament_target_dependencies(path_visualizer
  rclcpp
  geometry_msgs
  nav_msgs
)
install(TARGETS path_visualizer
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_rviz_panel)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()