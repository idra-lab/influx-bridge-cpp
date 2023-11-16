import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cmd = "ros2 run turtlesim turtle_teleop_key"
    subprocess.Popen(["gnome-terminal", "--", "bash", "-c", cmd])
    return LaunchDescription([
        Node(package="turtlesim", executable="turtlesim_node", output="screen"),
        Node(
            package="influxdb-bridge-cpp",
            executable="influxdb-bridge",
            output="screen"),
    ])

