from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='mail_delivery_robot',
             namespace='tests',
             executable='bumper_sensor_test',
             name='bumper_sensor_test',
             output='log',
             remappings=[('/tests/bumper', '/preceptions/bumper')]),
        Node(package='mail_delivery_robot',
             namespace='preceptions',
             executable='bumperSensor',
             name='bumperSensor',
             remappings=[('/preceptions/bumpEvent', '/tests/bumpEvent')]
             )
    ])



