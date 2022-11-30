#!/usr/bin/env python
# @author: Favour Olotu
from std_msgs.msg import String
from time import sleep
import rclpy
from rclpy.node import Node


# Test class for testing the throughput of the Robot Driver node
# Publishes a combination of preceptions, navigationMap and bumpEvent message to the robot driver node
# Subscribes to the actions topic to verify that the robot driver node processes properly
class RobotDriverNodeTest(Node):
    test_count = 0

    def __init__(self):
        super().__init__('robot_driver_test')

        # Create subscriber, msg_type = Bumper, topic = "bumper", callback = self.read_response
        self.subscriber = self.test_node.create_subscription(String, 'actions', self.callback, 10)
        self.perception_publisher = self.test_node.create_publisher(String, 'preceptions', 10)
        self.navigation_map_publisher = self.test_node.create_publisher(String, 'navigationMap', 10)
        self.bump_event_publisher = self.test_node.create_publisher(String, 'bumpEvent', 10)

        self.get_logger().info('Executing tests...')

        # Repeat test loop every 5 seconds
        timer_time = 5
        timer = self.create_timer(timer_time, self.test_throughput)

    # This function is called every time a message is received
    # TODO Need to update to ensure it matches new state machine
    def callback(self, message):

        # Check if returned values from actionTranslator are as expected
        if message.data == "":

            self.get_logger().info("TEST 1 PASSED")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        elif message.data == "":

            self.get_logger().info("TEST 2 PASSED")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        elif message.data == "":

            self.get_logger().info("TEST 3 PASSED")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        elif message.data == "":

            self.get_logger().info("TEST 4 PASSED")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        else:
            self.get_logger().info(
                f"TEST FAILED - INVALID Bumper message for {self.message}")

    def test_throughput(self):

        # TODO need to extract the exact message received
        perception_message = String()
        navigation_map_message = String()
        bump_event_message = String()

        perception_message.data = ""
        navigation_map_message.data = ""
        bump_event_message.data = ""

        # Run test one - Publishes command to actions topic
        if self.test_count == 0:
            self.get_logger().info(f"Sending a message")

            perception_message.data = ""
            navigation_map_message.data = ""
            bump_event_message.data = ""

            self.perception_publisher.publish(perception_message)
            self.navigation_map_publisher.publish(navigation_map_message)
            self.bump_event_publisher.publish(bump_event_message)

        # When test one completed, run test two
        elif self.test_count == 1:
            self.get_logger().info(f"Sending a message")

            perception_message.data = ""
            navigation_map_message.data = ""
            bump_event_message.data = ""

            self.perception_publisher.publish(perception_message)
            self.navigation_map_publisher.publish(navigation_map_message)
            self.bump_event_publisher.publish(bump_event_message)

        # When test one completed, run test two
        elif self.test_count == 2:
            self.get_logger().info(f"Sending a message")

            perception_message.data = ""
            navigation_map_message.data = ""
            bump_event_message.data = ""

            self.perception_publisher.publish(perception_message)
            self.navigation_map_publisher.publish(navigation_map_message)
            self.bump_event_publisher.publish(bump_event_message)

        # When test one completed, run test two
        elif self.test_count == 3:
            self.get_logger().info(f"Sending a message")

            perception_message.data = ""
            navigation_map_message.data = ""
            bump_event_message.data = ""

            self.perception_publisher.publish(perception_message)
            self.navigation_map_publisher.publish(navigation_map_message)
            self.bump_event_publisher.publish(bump_event_message)

        else:

            self.get_logger().info("TESTS PASSED SUCCESSFULLY")

            # Exit
            self.destroy_node()


# Initialize node
def main(args=None):
    rclpy.init()

    test_node = RobotDriverNodeTest()

    # Run Node on interval specified in the __init__ function, until it is killed
    rclpy.spin(test_node)

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
