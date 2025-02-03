from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    fly_square_node = Node(
        package="eo_drone",
        executable="fly_square",
        name="fly_square",
        output="screen",
    )
    depth_seeker_node = Node(
        package="eo_drone",
        executable="depth_seeker",
        name="depth_seeker",
        output="screen",
        parameters=[{"max_yaw_rate": 0.5}],  # rad/s
        remappings=[("~/avg_inv_depth", "/depth_to_bins/avg_inv_depth")],  # to, from
    )
    depth_to_bins_container = ComposableNodeContainer(
        name="depth_to_bins_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depth_utils",
                plugin="DepthToBinsNode",
                name="depth_to_bins",
                parameters=[{"num_bins": 8}],  # keep 8, else also change in depth_seeker.hpp
                # extra_arguments=[{"use_intra_process_comms": True}],  # only if publisher also supports this
                remappings=[("~/depth_image", "/depth_camera")],  # from /depth_camera to /depth_to_bins/depth_image
            ),
        ],
        output="screen",
    )
    return LaunchDescription([fly_square_node, depth_seeker_node, depth_to_bins_container])
