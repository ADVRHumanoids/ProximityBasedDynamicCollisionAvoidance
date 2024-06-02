import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  config = os.path.join(
      get_package_share_directory('dyn_collision_avoid'),
      'config',
      'dynamic_collision_avoidance_settings.yaml'
      )
  return LaunchDescription([
      Node(
            package='dyn_collision_avoid',
            executable='DynamicCollisionAvoidance',
            namespace='robot',
            name='dyn_coll_avoid_settings',
            parameters=[config],
            output="screen",
            emulate_tty=True
          )
    ])