colcon build --packages-select my_package

source install/local_setup.bash

ros2 run my_package my_node

ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package

rosdep install --from-paths src -y --ignore-src

sim_vehicle.py -v copter --console --map -w

STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5

gz sim -v4 -r iris_runway.sdf

sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console

ros2 run ros_gz_bridge parameter_bridge /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image


ros2 launch mavros apm.launch fcu_url:=udp://:14550@

