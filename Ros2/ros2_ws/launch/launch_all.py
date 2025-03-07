from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='action_tutorials_cpp',
            namespace='action_tu',
            executable='fibonacci_action_server',
            name='sim'
        ),
        Node(
            package='cpp_pubsub',
            namespace='pubsub',
            executable='talker',
            name='sim'
        ),
        Node(
            package='cpp_srvcli',
            namespace='service',
            executable='server',
            name='sim'
        ),
        Node(
            package='three_int_add',
            namespace='three_add',
            executable='server',
            name='sim'
        )
    ])