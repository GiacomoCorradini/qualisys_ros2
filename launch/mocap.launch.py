from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="qualisys_ros",
            executable="qualisys_node",
            name="qualisys_node",
            output="screen",
            emulate_tty=True,
            arguments=[
                "--server", "172.131.40.200",
                "--rate", "100",
            ],
        )
    ])