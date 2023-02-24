from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='mail_delivery_robot',
             namespace='tests',
             executable='captain_test',
             name='captain_test',
             output='log',
             remappings=[('/tests/beacons', '/navigation/beacons'),
                         ('/tests/navigator', '/navigation/navigator')]),
        Node(package='mail_delivery_robot',
             namespace='navigation',
             executable='captain',
             name='captain',
             remappings=[('/navigation/navigationMap', '/tests/navigationMap'),
                         ('/navigation/localMap', '/tests/localMap')]
             )
    ])



