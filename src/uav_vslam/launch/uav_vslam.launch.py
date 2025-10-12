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
    
    pkg_name_arg = DeclareLaunchArgument(
        'pkg_name',
        default_value='uav_vslam',
        description='The name of the package containing the Launch and RViz files.'
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
            'gz_args': '-v4 -r iris_runway.sdf'
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

    ardupilot_gazebo_folder = '/home/ubuntu/gz_ws/src/ardupilot_gazebo'
    uav_sdf_file = os.path.join(ardupilot_gazebo_folder, 'models', 'iris_with_gimbal', 'model.sdf')
    with open(uav_sdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Joint state publisher GUI node
    joint_state_publisher_gui = Node(
     package='joint_state_publisher_gui',
     executable='joint_state_publisher_gui',
     name='joint_state_publisher_gui',
     arguments=[uav_sdf_file],
     output=['screen']
    )

    # Robot state publisher node
    robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='both',
    parameters=[
        {'use_sim_time': True},
        {'robot_description': robot_desc},
    ]
)

    
    # ROS-Gazebo bridge node
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'config',
                'ros_gz_bridge.yaml'
            ])
        }]
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
    
    return LaunchDescription([
        pkg_name_arg,
        gz_sim,
        mavros,
        robot_state_publisher,
        ros_gz_bridge,
        rviz,
        set_gimbal_pitch
    ])