import launch
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    cam_name = LaunchConfig("camera_name").perform(context)
    subsample = int(LaunchConfig("subsample").perform(context))
    subsample_enabled = subsample > 0
    recording_container = ComposableNodeContainer(
        name="recording_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="libcaer_driver",
                plugin="libcaer_driver::Driver",
                name=cam_name,
                parameters=[
                    {
                        "device_type": LaunchConfig("device_type"),
                        "device_id": 1,
                        "encoding": "libcaer_cmp",
                        "master": True,
                        "serial": "",
                        "statistics_print_interval": 2.0,
                        "camerainfo_url": "",
                        "frame_id": "",
                        "event_message_time_threshold": 1.0e-3,
                        "event_message_size_threshold": 0,  # force immediate publish
                        "dvs_enabled": True,
                        "imu_accel_enabled": True,
                        "imu_gyro_enabled": True,
                        "subsample_enabled": subsample_enabled,
                        "subsample_horizontal": subsample,
                        "subsample_vertical": subsample,
                        "bias_sensitivity": LaunchConfig("bias_sensitivity"),
                        "time_reset_delay": 2,
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="rosbag2_composable_recorder",
                plugin="rosbag2_composable_recorder::ComposableRecorder",
                name="recorder",
                parameters=[
                    {
                        "topics": [
                            f"/{cam_name}/events",
                            f"/{cam_name}/imu",
                            "/fmu/out/actuator_motors",
                            "/fmu/out/hover_thrust_estimate",
                            "/fmu/out/vehicle_odometry",
                            "/fmu/out/vehicle_attitude",
                            "/fmu/out/vehicle_angular_velocity",
                        ],
                        "max_cache_size": 100 * 1024 * 1024,
                        "storage_id": "mcap",
                        "start_recording_immediately": True,
                        "bag_name": "",
                        "bag_prefix": "ros/rosbags/rosbag2_",
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )
    return [recording_container]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            # libcaer_driver
            LaunchArg(
                "camera_name",
                default_value=["event_camera"],
                description="camera name",
            ),
            LaunchArg(
                "device_type",
                default_value=["dvxplorer"],
                description="device type (davis, dvxplorer...)",
            ),
            LaunchArg(
                "subsample",
                default_value=["3"],
                description="subsampling step; 0 to disable, 1 for 2x, 3 for 4x",
            ),
            LaunchArg(
                "bias_sensitivity",
                default_value=["0"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
