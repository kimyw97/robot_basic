from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'ros2_test_1'
    urdf_name = 'T2503050025B774D6C95.urdf'
    rviz_config_file_name = 'testbot_edu.rviz'

    ld = LaunchDescription()

    # URDF 파일 경로 설정
    urdf_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', urdf_name)

   # URDF 파일 직접 읽기
    try:
        with open(urdf_path, 'r') as urdf_file:
            urdf_content = urdf_file.read()
    except Exception as e:
        print(f"Error reading URDF file: {e}")
        return ld

    # RViz 설정 파일 경로 설정
    rviz_config_path = os.path.join(get_package_share_directory(pkg_name), 'rviz', rviz_config_file_name)

    # robot_state_publisher 노드 실행
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_content}]
    )


    # joint_state_publisher_gui 노드 실행
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz2 실행
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path]
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz2_node)

    return ld