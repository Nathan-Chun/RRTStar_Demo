import os
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    urdf= os.path.join(
        get_package_share_directory('point_demo'),
        'urdf',
        'point.urdf'
    )
    world = os.path.join(
        get_package_share_directory("point_demo"),
        "worlds",
        "obstacle_world.world"
    )

    # Initialize gazebo
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items()
    )
    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'))
    # )

    # gz=ExecuteProcess(
    #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
    #     output='screen'
    # )

    # Spawn the sphere model in Gazebo
    spawn_point = ExecuteProcess(
        cmd = [
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-entity", "point",
            "-file", urdf,
            "-x", "1.0",
            "-y", "1.0",
            "-z", "1.0",
        ],  
        output = 'screen'
    )


    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)
    # ld.add_action(gz)
    ld.add_action(spawn_point)
    return ld


    # def move_sphere(self):
    #     msg = ModelState()
    #     msg.model_name = 'sphere'
    #     msg.pose.position.x = 1.0
    #     msg.pose.position.y = 1.0
    #     msg.pose.position.z = 1.0
    #     msg.pose.orientation.w = 1.0
    #     self.publisher.publish(msg)

