from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('my_tf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_diffbot.urdf')

    # 读取 URDF 文件内容
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        
    )

    tf_broadcaster = Node(
        package='my_tf',
        executable='my_tf2_broadcaster',
        name='my_tf2_broadcaster',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz)
    ld.add_action(tf_broadcaster)

    return ld
