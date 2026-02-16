from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression

def generate_launch_description():
    # Declare launch arguments
    pkg_name = LaunchConfiguration('pkg_name')
    world_name = LaunchConfiguration('world_name')
    gimbal_pitch = LaunchConfiguration('gimbal_pitch')
    gimbal_mode = LaunchConfiguration('gimbal_mode')
    
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

    gimbal_pitch_arg = DeclareLaunchArgument(
        'gimbal_pitch',
        default_value='90.0',
        description='The pitch angle of the gimbal in degrees.'
    )

    gimbal_mode_arg = DeclareLaunchArgument(
        'gimbal_mode',
        default_value='stabilized',
        description='Gimbal mode: "static" for fixed pitch or "stabilized" to run gimbal_leveler.py.'
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
            
            # Gimbal control commands
            '/gimbal/cmd_pitch@std_msgs/msg/Float64]gz.msgs.Double',
            '/gimbal/cmd_roll@std_msgs/msg/Float64]gz.msgs.Double',
            '/gimbal/cmd_yaw@std_msgs/msg/Float64]gz.msgs.Double',
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

    # Rviz Visualizer node
    rviz_visualizer = Node(
        package='extra_rviz_panel',
        executable='path_visualizer',
    )

    # Drone control node
    drone_control = Node(
        package='gps_denied_nav',
        executable='drone_control',
        parameters=[{
            'path_file': 'simple_path.yaml',
            'camera_pitch_angle': 60.0,
            'similarity_threshold': 0.6,
            'yaw_kp': 0.1,
            'pitch_kp': 0.1,
        }]
    )

    # Set Gimbal to the desired pitch angle (convert degrees to radians at runtime)
    # radians = degrees * pi / 180
    gimbal_pitch_radians = PythonExpression([
        gimbal_pitch, ' * 3.141592653589793 / 180.0'
    ])
    set_gimbal_pitch = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '/gimbal/cmd_pitch',
            'std_msgs/msg/Float64',
            ['{data: ', gimbal_pitch_radians, '}'],
            '--once'
        ],
        condition=IfCondition(PythonExpression(["'", gimbal_mode, "' == 'static'"])),
        output='screen'
    )

    gimbal_leveler = Node(
        package='gps_denied_nav',
        executable='gimbal_leveler.py',
        condition=IfCondition(PythonExpression(["'", gimbal_mode, "' == 'stabilized'"])),
        output='screen'
    )
    
    return LaunchDescription([
        pkg_name_arg,
        world_name_arg,
        gimbal_pitch_arg,
        gimbal_mode_arg,
        gz_sim,
        mavros,
        ros_gz_bridge,
        rviz,
        # drone_control,
        TimerAction(
            period=3.0,
            actions=[
                set_gimbal_pitch,
                gimbal_leveler,
                rviz_visualizer
            ]
        )
    ])
