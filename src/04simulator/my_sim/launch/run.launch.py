from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # 请修改以下路径，使其指向你的gazeboworld文件夹的路径
    gazeboworld_path = "/home/lehan/tutorial_ws/src/04simulator/gazeboworld"


    models_path = os.path.join(gazeboworld_path, "models")
    worlds_path = os.path.join(gazeboworld_path, "worlds")
    world_file = os.path.join(worlds_path, "rmuc_2025_world.sdf")
    
    environment = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path)
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '--verbose'],
        output='screen'
    )
    ld.add_action(environment)
    ld.add_action(gazebo)

    # 生成机器人
    pkg_path = get_package_share_directory('my_sim')        
    robot_path = os.path.join(pkg_path, "urdf", "my_diffbot.urdf")

    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'my_diffbot',
            '-file', robot_path,
            '-x', '5', '-y', '5', '-z', '0.8'
        ],
        output='screen'
    )


    with open(robot_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )


    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        
        arguments=[
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(bridge_lidar)
    return ld




