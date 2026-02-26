from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    run_loc_online = Node(
        package='lightning',
        executable='run_loc_online',
        name='run_loc_online',
        output='screen',
        arguments=[
                '--config', './src/lightning-lm/config/indoor_livox_walk.yaml'
            ],
    )

    static_tf_broadcaster = Node(
        package="lightning",
        executable="static_tf_broadcaster",
        name="static_tf_broadcaster",
        output="screen",
    )

    tf_transformer = Node(
        package="lightning",
        executable="tf_transformer",
        name="tf_transformer",
        output="screen",
    )

    globalmap_server = Node(
        package="lightning",
        executable="globalmap_server",
        name="globalmap_server",
        output="screen",
        parameters=[
            {'globalmap_pcd': '/home/kang/lightning_ws/data/eai_map/global.pcd'}
        ],
    )

    return LaunchDescription([
        # run_loc_online,
        static_tf_broadcaster,
        tf_transformer,
        globalmap_server
    ])