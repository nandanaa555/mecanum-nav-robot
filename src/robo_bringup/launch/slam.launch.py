from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    TimerAction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('robo_desc')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    xacro_file    = os.path.join(pkg_share, 'urdf',   'robot.xacro')
    rviz_config   = os.path.join(pkg_share, 'config', 'new.rviz')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge_gazebo_2.yaml')
    world = os.path.join(pkg_share, "world", "ware2.sdf")

    # ── Xacro → URDF ──────────────────────────────────────────────────────────
    robot_description = xacro.process_file(xacro_file).toxml()

    # ── Launch argument: show RViz? ────────────────────────────────────────────
    declare_rviz = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        description='Launch RViz2'
    )
    launch_rviz = LaunchConfiguration('rviz')

    mecanum_odom = Node(
        package='mecanum_odom',
        executable='odom_node',
        name='odom_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # 1. GAZEBO  — start first, everything else waits on it
    # ══════════════════════════════════════════════════════════════════════════
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': os.path.join(pkg_share, 'meshes')
        }
    )

    # ══════════════════════════════════════════════════════════════════════════
    # 2. ROBOT STATE PUBLISHER  — needs robot_description, no delay needed
    # ══════════════════════════════════════════════════════════════════════════
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # 3. SPAWN ROBOT  — wait 5 s for Gazebo to fully load the world
    # ══════════════════════════════════════════════════════════════════════════
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                arguments=[
                    '-topic', 'robot_description',
                    '-name',  'urdf_model',
                    '-allow_renaming', 'false',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.02',   # ← slightly above ground, prevents clipping
                    '-Y', '0.0',
                ],
                output='screen'
            )
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # 4. ROS ↔ GZ BRIDGE  — wait 6 s (after spawn)
    # ══════════════════════════════════════════════════════════════════════════
    bridge = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                parameters=[{
                    'config_file': bridge_config,
                    # keeps /tf_static alive for late RViz subscribers
                    'qos_overrides./tf_static.publisher.durability': 'transient_local',
                }]
            )
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # 5. RVIZ  — wait 9 s (after bridge is up)
    # ══════════════════════════════════════════════════════════════════════════
    rviz = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(launch_rviz)
            )
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # 6. SLAM TOOLBOX (ONLINE ASYNC)  — wait 10 s, includes the package's own
    #    online_async_launch.py instead of running the node directly, so it
    #    picks up slam_toolbox's default params/behavior automatically.
    #    Equivalent to:
    #      ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
    # ══════════════════════════════════════════════════════════════════════════
    slam_toolbox = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={'use_sim_time': 'True'}.items()
            )
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # 7. TELEOP  — wait 11 s, opens in its own terminal window since it needs
    #    live keyboard input (a plain Node has no interactive stdin).
    #    Equivalent to:
    #      ros2 run teleop_twist_keyboard teleop_twist_keyboard
    #    Uses gnome-terminal; the window stays open after Ctrl+C (exec bash).
    # ══════════════════════════════════════════════════════════════════════════
    teleop = TimerAction(
        period=11.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--', 'bash', '-c',
                    'ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        declare_rviz,

        # ordered by dependency
        gazebo,                   # 1 — starts immediately
        robot_state_publisher,    # 2 — starts immediately
        spawn_robot,              # 3 — fires at  5 s
        bridge,                   # 4 — fires at  6 s
        mecanum_odom,
        rviz,                     # 5 — fires at  9 s
        slam_toolbox,             # 6 — fires at 10 s
        teleop,                   # 7 — fires at 11 s
    ])
