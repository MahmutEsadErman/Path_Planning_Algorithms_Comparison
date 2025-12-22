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

ros2 bag record -o zigzag_yol /camera/image /camera/camera_info /simulation_pose_info /mavros/imu/data /mavros/global_position/rel_alt

ros2 bag record -o path_90degree /camera/image /camera/camera_info /simulation_pose_info /mavros/imu/data /mavros/global_position/rel_alt


sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --out=udp:127.0.0.1:14551

ros2 run gps_denied_nav drone_control
ros2 run gps_denied_nav main --ros-args -p similarity_threshold:=100 -p yaw_kp:=0.01











        // if (path_index_ > prev_index) {
        //     RCLCPP_INFO(this->get_logger(), "path_index_: %d", (int)path_index_);
        //     const FrameData &current_frame = path_data_[path_index_];
        //     prev_index = path_index_;
        //     // Convert quaternion to roll/pitch/yaw
        //     double qx = current_frame.imu.orientation.x;
        //     double qy = current_frame.imu.orientation.y;
        //     double qz = current_frame.imu.orientation.z;
        //     double qw = current_frame.imu.orientation.w;
        //     // Calculate yaw
        //     double siny_cosp = 2.0 * (qw * qz + qx * qy);
        //     double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        //     // target_yaw = std::atan2(siny_cosp, cosy_cosp);

        //     altitude = current_frame.altitude.data;

        //     vel_x = vel;
        // }
        // else {
        //     vel_x -= pitch_kp;
        // }

        // if (target_yaw > M_PI/2 || target_yaw < 3*M_PI/2) {
        //     pitch_kp = -pitch_kp;
        // }
        
        // vel_x = std::max(-5.0, std::min(5.0, vel_x));
        // prev_similarity = similarity;



