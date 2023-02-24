from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='mail_delivery_robot',
             namespace='tests',
             executable='robot_driver_test',
             name='robot_driver_test',
             output='log',
             remappings=[('/tests/preceptions', '/control/preceptions'),
                         ('/tests/navigationMap', '/control/navigationMap'),
                         ('/tests/bumpEvent', '/control/bumpEvent')]),
        Node(package='mail_delivery_robot',
             namespace='control',
             executable='robot_driver',
             name='robot_driver',
             remappings=[('/control/actions', '/tests/actions')]
             )
    ])



