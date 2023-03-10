#!/usr/bin/env python

# @author: Simon Yacoub, Devon Daley and Jacob Charpentier (built on top of previous year's work)

# SUBSCRIBER:   String object from 'actions' node
# PUBLISHER:    Twist object to 'cmd_vel' node
#               null object to 'dock' node
import math
import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time
import csv
import os
# This script is meant to take all the action decisions from our reasoner and publish them to the roomba (via cmd_vel)


# ~~~~ Load overrides ~~~~
def loadNumberOverrides():
    magicNumbers = {}
    ROOT_DIR = os.getcwd()
    with open(f'{ROOT_DIR}/src/carleton-mail-delivery-robot/mail_delivery_robot/magicNumbers.csv') as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        for row in reader:
            magicNumbers[row[0]] = row[1]
    return magicNumbers


magicNumbers = loadNumberOverrides()

class ActionTranslator(Node):
    def __init__(self):
        super().__init__('action_translator')
        self.drivePublisher = self.create_publisher(Twist, 'cmd_vel', 2)
        #unimplemented docking behaviour 
        self.undockPublisher = self.create_publisher(Empty, 'dock', 1)
        self.dockPublisher = self.create_publisher(Empty, 'undock', 1)
        
        self.subscription = self.create_subscription(String, 'actions', self.decodeAction, 10)

    # Decode and execute the action
    def decodeAction(self, data):
        action = str(data.data)
    

        data_list = action.split(":")
        self.get_logger().info(str(data_list)) 
        message = Twist() 
        if len(data_list) == 1:
            #this is for single paranmeter commandsi
            pass        
        elif len(data_list) == 2:
            #this is for just linear and angular messagei

            message.angular.z = float(data_list[1]) 
            message.linear.x = float(data_list[0]) 

            self.get_logger().info(data.data)
        elif len(data_list) == 3:
            #this is for target distance, cur distance, cur angle, 
            target_distance, current_distance, current_angle = data_list
            time_interval = float(magicNumbers['TIMER_PERIOD'])

            self.get_logger().info('distance: ' + current_distance)
            self.get_logger().info('angle: ' + current_angle)
            self.get_logger().info('target distance: ' + target_distance)

            message.angular.z = float(target_distance) * 2
            message.linear.x = float(magicNumbers['FORWARD_X_SPEED']) 

            # actionMessage = Twist()  # the mess
            # handle basic movement commands from actions topic
            self.get_logger().info("angular.z: " + str(message.angular.z) + " ||| linear.x: " + str(message.linear.x))
        else:
            #this is an unexpected message
            pass
        self.drivePublisher.publish(message)

# Get a Twist message which consists of a linear and angular component which can be negative or positive.
#
# linear.x  (+)     Move forward (m/s)
#           (-)     Move backward (m/s)
#
# angular.z (+)     Rotate counter-clockwise (rad/s)
#          (-)     Rotate clockwise (rad/s)
#
# Limits:
# -0.5 <= linear.x <= 0.5 and -4.25 <= angular.z <= 4.25 (4rads = 45deg)

# Main execution
def main():
    rclpy.init()
    action_translator = ActionTranslator()
    rclpy.spin(action_translator)


# Start things up
if __name__ == '__main__':
    main()
