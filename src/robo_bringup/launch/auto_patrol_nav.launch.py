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
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    xacro_file    = os.path.join(pkg_share, 'urdf',   'robot.xacro')
    rviz_config   = os.path.join(pkg_share, 'config', 'nav.rviz')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge_gazebo_2.yaml')

    map_yaml = "/home/haha/v_hack_ws/src/robo_bringup/maps/my_map.yaml"

    world = os.path.join("/home/haha/v_hack_ws/src/robo_desc/world/ware2.sdf")
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
    # 5. JOINT STATE PUBLISHER  — wait 7 s
    #
    #    WHY: Gazebo's JointStatePublisher plugin publishes to a world-scoped
    #    gz topic. The bridge forwards it to /joint_states. HOWEVER there is
    #    a race where RSP starts before /joint_states arrives, leaving wheels
    #    at zero position in RViz.
    #
    #    This node listens to /joint_states (from bridge) and re-publishes,
    #    ensuring RSP always gets a fresh message.
    #    Set use_sim_time=True so timestamps match Gazebo clock.
    # ══════════════════════════════════════════════════════════════════════════
    joint_state_publisher = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    # tell JSP to mirror what comes from the bridge
                    'source_list': ['/joint_states'],
                    'rate': 50
                }]
            )
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # 6. RVIZ  — wait 9 s
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
    # 7. NAV2 BRINGUP  — wait 10 s (after RViz/bridge are up), includes the
    #    package's own bringup_launch.py.
    #    Equivalent to:
    #      ros2 launch nav2_bringup bringup_launch.py \
    #           map:=/home/haha/v_hack_ws/map.yaml use_sim_time:=true
    # ══════════════════════════════════════════════════════════════════════════
    nav2_bringup = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_yaml,
                    'use_sim_time': 'true',
                }.items()
            )
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # 8. AUTO PATROL  — wait 12 s (after nav2 bringup has had time to come up)
    #    Equivalent to:
    #      ros2 run auto_patrol patrol
    # ══════════════════════════════════════════════════════════════════════════
    auto_patrol = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='auto_patrol',
                executable='patrol',
                name='auto_patrol',
                output='screen',
                parameters=[{
                    'use_sim_time': True
                }]
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
        #joint_state_publisher,    # 5 — fires at  7 s
        mecanum_odom,

        rviz,                     # 6 — fires at  9 s
        nav2_bringup,             # 7 — fires at 10 s
        auto_patrol,              # 8 — fires at 12 s
    ])