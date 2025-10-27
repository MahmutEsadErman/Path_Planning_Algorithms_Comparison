import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Declare launch arguments
    pkg_name = LaunchConfiguration('pkg_name')
    world_name = LaunchConfiguration('world_name')
    
    pkg_name_arg = DeclareLaunchArgument(
        'pkg_name',
        default_value='gps_denied_nav',
        description='The name of the package containing the Launch and RViz files.'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='iris_runway',
        description='The name of the Gazebo world to launch.'
    )

    # Launch Gazebo with the iris_runway world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [' -r --headless-rendering ', world_name, '.sdf']
        }.items()
    )
    
    # Start the MAVROS node
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'apm.launch'
            ])
        ]),
        launch_arguments={
            'fcu_url': 'udp://:14550@'
        }.items()
    )

    # ardupilot_gazebo_folder = '/home/ubuntu/ardupilot_gazebo'
    # uav_sdf_file = os.path.join(ardupilot_gazebo_folder, 'models', 'iris_with_gimbal', 'model.sdf')
    # with open(uav_sdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    # Robot state publisher node
#     robot_state_publisher = Node(
#     package='robot_state_publisher',
#     executable='robot_state_publisher',
#     name='robot_state_publisher',
#     output='both',
#     parameters=[
#         {'use_sim_time': True},
#         {'robot_description': robot_desc},
#     ]
# )

    # ROS-Gazebo bridge node
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock synchronization
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            
            # Pose information
            ['/world/', world_name, '/pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'],
            
            # Camera image from gimbal
            ['/world/', world_name, '/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
            
            # Camera info
            ['/world/', world_name, '/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
            
            # IMU data
            ['/world/', world_name, '/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            
            # Keyboard control input
            '/keyboard/keypress@std_msgs/msg/Int32[gz.msgs.Int32',
            
            # Gimbal control commands - Pitch
            '/gimbal/cmd_pitch@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            # Remap Gazebo topics to cleaner ROS topic names
            (['/world/', world_name, '/pose/info'], '/simulation_pose_info'),
            (['/world/', world_name, '/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'], '/camera/image'),
            (['/world/', world_name, '/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/camera_info'], '/camera/camera_info'),
            (['/world/', world_name, '/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu'], 'drone/imu'),
        ]
    )
    
    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'config',
                'drone_sim.rviz'
            ])
        ]
    )

    # Set Gimbal to 60 degrees down
    set_gimbal_pitch = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '/gimbal/cmd_pitch',
            'std_msgs/msg/Float64',
            '{data: 1.0472}', # 60 degrees in radians 
            '--once'
        ],
        output='screen'
    )

    # static_tf_example = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments=[
    #             '--x', '0', '--y', '0', '--z', '1',
    #             '--yaw', '0', '--pitch', '0', '--roll',
    #             '0', '--frame-id', 'world', '--child-frame-id', 'mystaticturtle']
    #     ),
    
    return LaunchDescription([
        pkg_name_arg,
        world_name_arg,
        gz_sim,
        mavros,
        # robot_state_publisher,
        ros_gz_bridge,
        rviz,
        set_gimbal_pitch
    ])