#!/usr/bin/env python
#@author: Emmitt Luhning
import unittest
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from time import sleep
import rclpy
from rclpy.node import Node 

# Test class for testing the actionTranslator Node
# Publishes each of the types of actions avaialble to simulate the robotDriver
# Subscribes to cmd_vel to verify that the robot controller is recieving the correct data for given command
class ActionTranslatorTest(Node):

    test_count = 0
    current_message = "left" 

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
    
    def __init__(self):
        super().__init__('action_translator_test')
        
        ##Create publisher and subscriber
        self.action_publisher = self.create_publisher(String, 'actions', 2)
        self.subscriber = self.create_subscription(Twist, 'cmd_vel', self.callback, 10) 
        
   
        self.get_logger().info('Executing tests...')

        ##Repeat test loop every 5 seconds
        timer_time = 5
        timer = self.create_timer(timer_time, self.test_decode_action)


    # ~~~~ TODO: Load Magic NUmbers from CSV ~~~~
    #def loadNumberOverrides():
    #    with open('/var/local/magicNumbers.csv') as csvfile:
    #        reader = csv.reader(csvfile, delimiter=",")
    #        for row in reader:
    #            magicNumbers[row[0]] = row[1]
    #    return magicNumbers
    
    ##Function called when the subcriber hears a message on the cmd_vel topic that actionTranslator publishes to
    def callback(self, twist_message):

        ##Check if returned values from actionTranslator are as expected 
        if ( self.current_message == 'left' and twist_message.linear.x == float(self.magicNumbers['ZERO_SPEED']) 
                and twist_message.angular.z == float(self.magicNumbers['LEFT_Z_SPEED'])):
            
            self.left_message_properly_proccesed = True
            self.get_logger().info("TEST 1 PASSED: Left movement twist values as expected")
            ##Increment test count so next loop runs test 2
            self.test_count = 1

        elif(self.current_message == 'right'
                and twist_message.linear.x == float(self.magicNumbers['ZERO_SPEED']) 
                and twist_message.angular.z == float(self.magicNumbers['RIGHT_Z_SPEED'])):
            
            self.get_logger().info("TEST 2 PASSED: right movement twist values as expected")
            ##Increment test count so next loop runs test 3
            self.test_count = 2
        elif(self.current_message == 'stop'
                and twist_message.linear.x == float(self.magicNumbers['ZERO_SPEED'])
                and twist_message.angular.z == float(self.magicNumbers['ZERO_SPEED'])):
            
            self.get_logger().info("TEST 3 PASSED: stop movement twist values as expected")
            self.test_count = 3
        else:
            self.get_logger().info(f"TEST FAILED - INVALID TWiST VALUES for {self.current_message} {twist_message.linear.x}, {twist_message.angular.z}")
        
    def test_decode_action(self):
        
        action_message = String()

        #Run test one - Publishes command to actions topic
        if self.test_count == 0:
            self.get_logger().info(f"Sending Left")
            
            action_message.data = "left"
            self.action_publisher.publish(action_message)

        ##When test one completed, run test two
        elif self.test_count == 1:
            self.get_logger().info(f"Sending Right")   
            self.current_message = "right" 
            action_message.data = "right"
            self.action_publisher.publish(action_message)

        ##When test twp completed, run test three
        elif self.test_count == 2:
            self.get_logger().info(f"Sending Stop")  
            self.current_message = "stop"  
            action_message.data = "stop"
            self.action_publisher.publish(action_message)
        else:
            
            self.get_logger().info("TESTS PASSED SUCESSFULLY")
            
            #Exit
            self.destroy_node()
        
#Initialize node
def main(args = None):
        rclpy.init()
        
        test_node = ActionTranslatorTest()

        #Run Node on interval specified in the __init__ function, until it is killed
        rclpy.spin(test_node) 
        
        test_node.destroy_node() 
        rclpy.shutdown() 

if __name__ == '__main__':
    main()
