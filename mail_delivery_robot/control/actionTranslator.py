#!/usr/bin/env python

# @author: Simon Yacoub and Devon Daley (built on top of previous year's work)

# SUBSCRIBER:   String object from 'actions' node
# PUBLISHER:    Twist object to 'cmd_vel' node
#               null object to 'dock' node

import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time
import csv

# This script is meant to take all the action decisions from our reasoner and publish them to the roomba (via cmd_vel)


# ~~~~ DEFAULTS ~~~~~
magicNumbers = {
    'ZERO_SPEED': 0.0,
    'FORWARD_X_SPEED': 0.2,
    'SLOW_FORWARD_X_SPEED': 0.1,
    'CREEP_FORWARD_X_SPEED': 0.05,
    'BACKWARD_X_SPEED': -0.2,
    'LEFT_Z_SPEED': 3.5,
    'RIGHT_Z_SPEED': -3.5,
    'SLEFT_X_SPEED': 0.05,
    'SLEFT_Z_SPEED': 0.5,
    'SRIGHT_X_SPEED': 0.05,
    'SRIGHT_Z_SPEED': -0.5,
    'AVOIDRIGHT_X_SPEED': 0.08,
    'AVOIDRIGHT_Z_SPEED': -0.5,
    'BLEFT_X_SPEED': -0.1,
    'BLEFT_Z_SPEED': 0.5,
}

# ~~~~ Load overrides ~~~~
def loadNumberOverrides():
    with open('/var/local/magicNumbers.csv') as csvfile:
        reader = csv.reader(csvfile,delimiter=",")
        for row in reader:
            magicNumbers[row[0]]= row[1]
    return magicNumbers

class ActionTranslator(Node):
    def __init__(self):
        super().__init__('action_translator')
        self.drivePublisher = self.create_publisher(Twist,'cmd_vel',2)
        self.undockPublisher = self.create_publisher(Empty,'dock',1)
        self.dockPublisher = self.create_publisher(Empty,'undock',1)

        self.subscription = self.create_subscription(String,'actions',self.decodeAction,10)


    # Decode and execute the action
    def decodeAction(self, data):
        actionMessage = Twist() #the mess

        # Get the parameters
        action = str(data.data)
        # (drivePublisher, dockPublisher, undockPublisher) = args
        
        #handle basic movement commands from actions topic
        actionMessage = getTwistMesg(action)
        if(action == "left"): #Does a 45 degree turn left (stops robot first)
            actionMessage = getTwistMesg("left")
            tmp = String()
            tmp.data = "stop"
            self.drivePublisher.publish(actionMessage)
            self.get_logger().info("Action: left")

        elif(action == "right"): #Does 45 degree turn right (stops robot first)
            actionMessage = getTwistMesg("right")
            tmp = String()
            tmp.data = "stop"
            self.drivePublisher.publish(actionMessage)
            self.get_logger().info("Action: right")

        # Handle the docking station cases
        if action == "dock":
            self.dockPublisher.publish(Empty())
        elif action == 'undock':
            self.undockPublisher.publish(Empty())
        else:
            #publish action
            self.drivePublisher.publish(actionMessage)
        
        
'''
Get a Twist message which consists of a linear and angular component which can be negative or positive.

linear.x  (+)     Move forward (m/s)
          (-)     Move backward (m/s)

angular.z (+)     Rotate counter-clockwise (rad/s)
         (-)     Rotate clockwise (rad/s)

Limits:
-0.5 <= linear.x <= 0.5 and -4.25 <= angular.z <= 4.25 (4rads = 45deg)
'''
def getTwistMesg(action):
    message = Twist()
    
    if action == "forward":
        message.linear.x = float(magicNumbers['FORWARD_X_SPEED'])
        message.angular.z = float(magicNumbers['ZERO_SPEED'])
    elif action == "slowForward":
        message.linear.x = float(magicNumbers['SLOW_FORWARD_X_SPEED'])
        message.angular.z = float(magicNumbers['ZERO_SPEED'])
    elif action == "creepForward":
        message.linear.x = float(magicNumbers['CREEP_FORWARD_X_SPEED'])
        message.angular.z = float(magicNumbers['ZERO_SPEED'])
    elif action == "backward":
        message.linear.x = float(magicNumbers['BACKWARD_X_SPEED'])
        message.linear.z = float(magicNumbers['ZERO_SPEED'])
    elif action == "left":
        message.linear.x = float(magicNumbers['ZERO_SPEED'])
        message.angular.z = float(magicNumbers['LEFT_Z_SPEED'])
    elif action == "right":
        message.linear.x = float(magicNumbers['ZERO_SPEED'])
        message.angular.z = float(magicNumbers['RIGHT_Z_SPEED'])
    elif action == "sleft":
        message.linear.x = float(magicNumbers['SLEFT_X_SPEED'])
        message.angular.z = float(magicNumbers['SLEFT_Z_SPEED'])
    elif action == "sright":
        message.linear.x = float(magicNumbers['SRIGHT_X_SPEED'])
        message.angular.z = float(magicNumbers['SRIGHT_Z_SPEED'])
    elif action == "avoidright":
        message.linear.x = float(magicNumbers['AVOIDRIGHT_X_SPEED'])
        message.angular.z = float(magicNumbers['AVOIDRIGHT_Z_SPEED'])
    elif action == "bleft":
        message.linear.x = float(magicNumbers['BLEFT_X_SPEED'])
        message.angular.z = float(magicNumbers['BLEFT_Z_SPEED'])
    elif action == "stop":
        message.linear.x = float(magicNumbers['ZERO_SPEED'])
        message.angular.z = float(magicNumbers['ZERO_SPEED'])
    
    return message

# Main execution
def main():
    try:
        loadNumberOverrides()
    except:
        print("No tuning file found!")
    rclpy.init()
    action_translator = ActionTranslator()
    rclpy.spin(action_translator)

# Start things up
if __name__ == '__main__':
    main()
