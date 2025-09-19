colcon build --packages-select my_package

source install/local_setup.bash

ros2 run my_package my_node

ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package

rosdep install --from-paths src -y --ignore-src