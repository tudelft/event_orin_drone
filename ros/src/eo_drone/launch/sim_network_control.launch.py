from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    monodepth_node = Node(
        package="depth_estimation",
        executable="monodepth",
        output="screen",
        name="monodepth",
        remappings=[("~/image", f"/camera")],  # to, from
    )
    depth_seeker_node = Node(
        package="eo_drone",
        executable="depth_seeker",
        name="depth_seeker",
        output="screen",
        parameters=[
            {
                "mode_name": "DS Monodepth",
                "navigation_state": "ext1",
                "max_yaw_rate": 0.5,  # rad/s
            }
        ],
        remappings=[("~/avg_inv_depth", "/monodepth/avg_inv_depth")],  # to, from
    )
    return LaunchDescription([monodepth_node, depth_seeker_node])
