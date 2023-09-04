from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    ld = LaunchDescription()

    tennis_ball_publisher_node = Node(
        package="tennis_ball_tracker",
        executable="tennis_ball_publisher",
        parameters=[
            {"input_file": "tennis-ball-video.mp4"}
        ]
    )

    tennis_ball_listener_node = Node(
        package="tennis_ball_tracker",
        executable="tennis_ball_listener"
    )

    ld.add_action(tennis_ball_publisher_node)
    ld.add_action(tennis_ball_listener_node)

    return ld
