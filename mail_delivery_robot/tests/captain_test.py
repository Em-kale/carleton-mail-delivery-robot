#!/usr/bin/env python
# @author: Favour Olotu
from std_msgs.msg import String
import rclpy
from rclpy.node import Node


# Test class for testing the throughput of the captain node
# Publishes a beacon message to the captain node
# Subscribes to navigation to verify that the captain processes a beacon information correctly
class CaptainNodeTest(Node):
    test_count = 0

    def __init__(self):
        super().__init__('captain_test')

        # Create publisher and subscriber
        self.publisher = self.create_publisher(String, 'navigator', 2)
        self.subscriber = self.create_subscription(String, 'navigationMap', self.callback, 10)

        self.get_logger().info('Executing tests...')

        # Repeat test loop every 5 seconds
        timer_time = 5
        timer = self.create_timer(timer_time, self.test_navigator_throughput)

    # This function is called every time a message is received
    def callback(self, message):

        # Check if returned values from actionTranslator are as expected
        if message.data == "straight":
            # Initial beacon message does not prompt any reply message
            self.get_logger().info("TEST 1 PASSED: Captain Navigation command as expected")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        elif message.data == "right":

            self.get_logger().info("TEST 2 PASSED: Captain Navigation command as expected")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        elif message.data == "u-turn":

            self.get_logger().info("TEST 3 PASSED: Captain Navigation command as expected")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        elif message.data == "dock":

            self.get_logger().info("TEST 4 PASSED: Captain Navigation command as expected")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        else:
            self.get_logger().info(
                f"TEST FAILED - INVALID TWiST VALUES for {self.current_message} {message.linear.x}, {message.angular.z}")

    def test_navigator_throughput(self):

        message = String()

        # Run test one - Publishes command to actions topic
        if self.test_count == 0:
            self.get_logger().info(f"Sending -2 straight- navigation message")

            message.data = "2 straight"
            self.publisher.publish(message)

        # When test one completed, run test two
        elif self.test_count == 1:
            self.get_logger().info(f"Sending -2 right- navigation message")

            message.data = "2 right"
            self.publisher.publish(message)

        # When test two is completed, run test three
        elif self.test_count == 2:
            self.get_logger().info(f"Sending -2 u-turn- navigation message")

            message.data = "2 u-turn"
            self.publisher.publish(message)

        # When test two is completed, run test three
        elif self.test_count == 3:
            self.get_logger().info(f"Sending -2 destination- navigation message")

            message.data = "2 destination"
            self.publisher.publish(message)

        else:

            self.get_logger().info("TESTS PASSED SUCCESSFULLY")

            # Exit
            self.destroy_node()


# Initialize node
def main(args=None):
    rclpy.init()

    test_node = CaptainNodeTest()

    # Run Node on interval specified in the __init__ function, until it is killed
    rclpy.spin(test_node)

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
