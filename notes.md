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

ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

ros2 launch uav_vslam uav_vslam.launch.py


echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=/ros2_tutorials/new_models:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc

ros2 bag record -a -o simple_path2

ros2 bag record -o uzun_yol /camera/image /camera/camera_info /simulation_pose_info /mavros/imu/data /mavros/global_position/rel_alt

ros2 bag record -o yeni_harita /camera/image /camera/camera_info /simulation_pose_info /mavros/imu/data /mavros/global_position/rel_alt


# Başlangıç noktasını bizim okula ayarladım
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --out=udp:127.0.0.1:14551 -l 41.0258025,28.8884930,0,0


ros2 run gps_denied_nav drone_control

ros2 run gps_denied_nav main --ros-args -p yaw_kp:=0.05 -p pitch_kp:=5.0 -p path_file:=yeni_harita_SURF.yaml

ros2 run gps_denied_nav create_path --ros-args -p bag_file_path:=yeni_harita


ros2 run gps_denied_nav drone_control

message SET_GPS_GLOBAL_ORIGIN 0 41.0258025 28.8884930 600000 0

Guided (41.02039580875228, 28.88856993578641) 50.0 frame 3




