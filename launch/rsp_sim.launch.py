import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration

# import xacro

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    # Specify the name of the package and path to xacro file within the package
    package_name = 'my_bot'
    # file_subpath = 'description/urdf/robot.urdf.xacro'
    package_path = get_package_share_directory(package_name)


    # Use xacro to process the file
    # xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    # robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    # node_robot_state_publisher = Node(
        # package='robot_state_publisher',
        # executable='robot_state_publisher',
        # output='screen',
        # parameters=[{'robot_description': robot_description_raw,
        # 'use_sim_time': True}] # add other parameters here if required
    # )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(package_path,'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(package_path,'launch','joystick.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # twist_mux_params = os.path.join(package_path,'config','twist_mux.yaml')
    # twist_mux = Node(
        # package="twist_mux",
        # executable="twist_mux",
        # namespace='/',
        # parameters=[twist_mux_params, {'use_sim_time': True}],
        # remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    # )

    # twist_stamper = Node(
        # package='twist_stamper',
        # executable='twist_stamper',
        # namespace='/',
        # use_sim_time must be False here, or time stamp will be 0:
        # parameters=[{'use_sim_time': False}, {'frame_id': 'base_link'}],
        # remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
                    # ('/cmd_vel_out','/diff_cont/cmd_vel')]
    # )

    # gazebo = IncludeLaunchDescription(
        # PythonLaunchDescriptionSource([os.path.join(
            # get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        # )

    # Start Gazebo Harmonic (GZ, Ignition)
    # -- set gazebo sim resource path for meshes and STLs:
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(package_path, 'worlds'), ':' +
            os.path.join(package_path, 'description')
            ]
        )

    # -- where to find the world SDF:
    gazebo_arguments = LaunchDescription([
            DeclareLaunchArgument('world', default_value='test_robot_world',    #<--- CHANGE THIS
                                  description='Gz sim Test World'),             #<--- CHANGE THIS
        ]
    )

    # -- how to launch Gazebo UI:
    gazebo_ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),
                '.sdf',
                ' -v 4',
                ' -r']
            )
        ]
    )

    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    # arguments=['-topic', 'robot_description',
                                # '-entity', 'my_bot'],
                    # output='screen')

    # spawn entity (robot model) in the Gazebo gz_sim:
    spawn_sim_robot = Node(package='ros_gz_sim',
        executable='create',
        namespace='/',
        arguments=[
            '-name', 'my_bot',			# changed from Sergei's 'dragger'
            '-topic', '/robot_description',
            '-allow_renaming', 'true'],
        parameters=[{'use_sim_time': True}],
        output='screen')

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace='/',
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace='/',
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        #arguments=["diff_cont", "--controller-manager", "/controller_manager", "--ros-args", "--remap",  "/diff_cont/odom:=/odomodom"],  # --ros-args --remap odom:=odomodom
        #remappings=[('/diff_cont/odom','/odom')]
    )

    # No ned to run controller_manager - it runs within Gazebo ROS2 Bridge.
    # Only configure controllers, after the robot shows up live in GZ:

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_sim_robot,
            on_start=[joint_broad_spawner],
        )
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_broad_spawner,
            on_start=[diff_drive_spawner],
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        namespace='/',
        #arguments=['-d', os.path.join(package_path, 'config', 'view_bot.rviz')],
        #arguments=['-d', os.path.join(package_path, 'config', 'map.rviz')],
        arguments=['-d', os.path.join(package_path, 'config', 'main.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='/',
        parameters=[{
            'config_file': os.path.join(package_path, 'config', 'gz_ros_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Gazebo controller_manager is not subject to renaming through parameters, so we use topic relay here:
    # odom_relay = Node(
        # package='topic_tools',
        # executable='relay',
        # namespace='/',
        # parameters=[{ 'input_topic': '/diff_cont/odom', 'output_topic': '/odom'}],
    # )

    gz_include = GroupAction(
        actions=[

            #SetRemap(src='/diff_cont/odom', dst='/odom'),
            gazebo_resource_path,
            gazebo_arguments,
            gazebo_ui,
            spawn_sim_robot,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            rviz,
            bridge,
            # odom_relay
        ]
    )

    # Run the node
    # Launch them all!
    return LaunchDescription([
        # gazebo,
        # node_robot_state_publisher,
        # spawn_entity
        rsp,
        joystick,
        # twist_mux,
        # twist_stamper,
        gz_include
    ])
