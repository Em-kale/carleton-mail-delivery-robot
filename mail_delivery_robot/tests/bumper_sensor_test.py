#!/usr/bin/env python
# @author: Favour Olotu
from std_msgs.msg import String
from create_msgs.msg import Bumper
import rclpy
from rclpy.node import Node


# Test class for testing the throughput of the Bumper Sensor node
# Publishes a bumper message to the Bumper sensor node
# Subscribes to bumpEvent to verify that the Bumper sensor node processes a bumper message properly
class BumperSensorNodeTest(Node):
    test_count = 0

    def __init__(self):
        super().__init__('test_bumper_sensor')

        # Create subscriber, msg_type = Bumper, topic = "bumper", callback = self.read_response
        self.subscriber = self.create_subscription(String, 'bumpEvent', self.read_response, 10)
        self.publisher = self.create_publisher(Bumper, 'bumper', 2)


        self.get_logger().info('Executing tests...')

        # Repeat test loop every 5 seconds
        timer_time = 5
        timer = self.create_timer(timer_time, self.test_throughput)

    # This function is called every time a message is received
    def callback(self, message):

        # Check if returned values from actionTranslator are as expected
        if message.data == "Cpressed":

            self.get_logger().info("TEST 1 PASSED")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        elif message.data == "Lpressed":

            self.get_logger().info("TEST 2 PASSED")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        elif message.data == "Rpressed":

            self.get_logger().info("TEST 3 PASSED")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        elif message.data == "unpressed":

            self.get_logger().info("TEST 4 PASSED")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        else:
            self.get_logger().info(
                f"TEST FAILED - INVALID Bumper message for {self.message}")

    def test_throughput(self):

        # The exact message received from a bumper topic
        message = Bumper()

        # Run test one - Publishes command to actions topic
        if self.test_count == 0:
            self.get_logger().info(f"Sending centre pressed bumper message")

            message.is_left_pressed = True
            message.is_right_pressed = True

            self.publisher.publish(message)

        # When test one completed, run test two
        elif self.test_count == 1:
            self.get_logger().info(f"Sending left pressed bumper message")

            message.is_left_pressed = True
            message.is_right_pressed = False
            self.action_publisher.publish(message)

        # When test one completed, run test two
        elif self.test_count == 2:
            self.get_logger().info(f"Sending right pressed bumper message")

            message.is_left_pressed = False
            message.is_right_pressed = True
            self.action_publisher.publish(message)

        # When test one completed, run test two
        elif self.test_count == 3:
            self.get_logger().info(f"Sending Unpressed bumper message")

            message.is_left_pressed = False
            message.is_right_pressed = False
            self.action_publisher.publish(message)

        else:

            self.get_logger().info("TESTS PASSED SUCCESSFULLY")

            # Exit
            self.destroy_node()


# Initialize node
def main(args=None):
    rclpy.init()

    test_node = BumperSensorNodeTest()

    # Run Node on interval specified in the __init__ function, until it is killed
    rclpy.spin(test_node)

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
