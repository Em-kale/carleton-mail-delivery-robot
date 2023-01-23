from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='mail_delivery_robot',
             namespace='tests',
             executable='plot_navigation_test',
             name='plot_navigation_test',
             output='log',
             remappings=[('/tests/Requests', '/navigation/Requests'),
                         ('/tests/localMap', '/navigation/localMap')]),
        Node(package='mail_delivery_robot',
             namespace='navigation',
             executable='plot_navigation',
             name='plot_navigation',
             remappings=[('/navigation/navigator', '/tests/navigator')]
             )
    ])



