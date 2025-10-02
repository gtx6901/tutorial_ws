from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    config = PathJoinSubstitution([
        FindPackageShare('my_first_package'),
        'config',
        'params.yaml'
    ])

    node = Node(
        package='my_first_package',
        executable='helloworld',   # CMakeLists 中 add_executable 的 target 名
        name='my_publisher',
        output='screen',
        parameters=[config],
        emulate_tty=True
    )
    ld.add_action(node)
    return ld