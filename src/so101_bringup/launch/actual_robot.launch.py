import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    port_arg = DeclareLaunchArgument("port", default_value="/dev/ttyUSB0")
    read_only_arg = DeclareLaunchArgument(
        "read_only", default_value="false",
        description="true = only read the arm pose (torque stays off) - use this to check calibration",
    )

    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("so101_controller"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={
                "is_sim": "False",
                "port": LaunchConfiguration("port"),
                "read_only": LaunchConfiguration("read_only"),
            }.items()
        )

    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("so101_moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )

    return LaunchDescription([
        port_arg,
        read_only_arg,
        controller,
        moveit,
    ])
