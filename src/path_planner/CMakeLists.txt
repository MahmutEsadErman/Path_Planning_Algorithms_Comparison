cmake_minimum_required(VERSION 3.22)
project(path_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)

include_directories(include ${Qt6Widgets_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

qt_add_executable(qt_example
  src/main.cpp
  src/example_window.cpp
  src/grid_widget.cpp
  include/example_window.h
  include/grid_widget.hpp
  ui/example_window.ui
)

target_link_libraries(qt_example PRIVATE
  Qt6::Core
  Qt6::Gui
  Qt6::Widgets
  rclcpp::rclcpp
)

install(TARGETS qt_example
  DESTINATION lib/${PROJECT_NAME})

ament_package()
