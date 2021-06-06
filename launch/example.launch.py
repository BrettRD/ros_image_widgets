import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
def generate_launch_description():

    config_filename = launch.substitutions.LaunchConfiguration('config_filename')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('config_filename', default_value='compass_inline_template'),

        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('image_widgets'), 'config', '')),
            config_filename, launch.substitutions.TextSubstitution(text='.config.yaml')]),

        launch_ros.actions.Node(
            name='jinjaturtle',
            package='image_widgets',
            executable='jinjaturtle.py',
            parameters=[config_filepath]
        ),
    ])
