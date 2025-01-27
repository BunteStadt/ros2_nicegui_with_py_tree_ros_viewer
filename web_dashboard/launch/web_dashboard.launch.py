import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='web_dashboard',
            executable='dashboard',
            name='web_dashboard_node',
            output='screen',
            emulate_tty=True,
        )
    ])
