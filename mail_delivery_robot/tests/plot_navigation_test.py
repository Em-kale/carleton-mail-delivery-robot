#!/usr/bin/env python
# @author: Favour Olotu
from std_msgs.msg import String
import rclpy
from rclpy.node import Node


# Test class for testing the throughput of the plot Navigation node
# Publishes a combination of Requests and localMap messages to the plot navigator node
# Subscribes to the navigator topic to verify that the plot navigator node handles the messages properly
class PlotNavigationNodeTest(Node):
    test_count = 0

    def __init__(self):
        super().__init__('plot_navigation_test')

        # Create subscriber, msg_type = String, topic = "navigator", callback = self.callback
        self.captain_update_subscriber = self.create_subscription(String, 'navigator', self.callback, 10)
        self.request_publisher = self.create_publisher(String, 'Requests', 10)
        self.captain_update_publisher = self.create_publisher(String, 'localMap', 10)

        self.get_logger().info('Executing tests...')

        # Repeat test loop every 5 seconds
        timer_time = 5
        timer = self.create_timer(timer_time, self.test_throughput)
        self.test_count = 0

    # This function is called every time a message is received
    def callback(self, message):

        # Check if returned values from actionTranslator are as expected
        if message.data == "initial straight":

            self.get_logger().info("TEST 0 PASSED")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        elif message.data == "2 straight":

            self.get_logger().info("TEST 1 PASSED")
            # Increment test count so next loop runs test 3
            self.test_count += 1

        elif message.data == "3 right":

            self.get_logger().info("TEST 2 PASSED")
            # Increment test count so next loop runs test 4
            self.test_count += 1

        elif message.data == "4 straight":

            self.get_logger().info("TEST 3 PASSED")
            # Increment test count so next loop runs test 2
            self.test_count += 1

        elif message.data == "4 u-turn":

            self.get_logger().info("TEST 4 PASSED")
            # Increment test count so next loop runs test 3
            self.test_count += 1

        elif message.data == "4 right":

            self.get_logger().info("TEST 5 PASSED")
            # Increment test count so next loop runs test 4
            self.test_count += 1

        elif message.data == "5 destination":

            self.get_logger().info("TEST 6 PASSED")
            # Increment test count so next loop runs test 4
            self.test_count += 1

        else:
            self.get_logger().info(
                f"TEST FAILED - INVALID messages received")

    def test_throughput(self):

        server_request_message = String()
        local_map_message = String()

        # Run test one - Publishes to request publisher
        if self.test_count == 0:
            self.get_logger().info(f"Sending server request message")

            server_request_message.data = "1 5"

            self.request_publisher(server_request_message)

        # When test two completed, run test two
        elif self.test_count == 1:
            self.get_logger().info(f"Sending beacon 2 reached message")

            local_map_message.data = "2 reached"

            self.captain_update_publisher(local_map_message)

        # When test three completed, run test two
        elif self.test_count == 2:
            self.get_logger().info(f"Sending beacon 3 reached message")

            local_map_message.data = "3 reached"

            self.captain_update_publisher(local_map_message)

        elif self.test_count == 3:
            self.get_logger().info(f"Sending beacon 4 reached message")

            local_map_message.data = "4 reached"

            self.captain_update_publisher(local_map_message)

        elif self.test_count == 4:
            self.get_logger().info(f"Sending beacon 4 passed message")

            local_map_message.data = "4 Passed"

            self.captain_update_publisher(local_map_message)

        elif self.test_count == 5:
            self.get_logger().info(f"Sending beacon 4 reached message again")

            local_map_message.data = "4 reached"

            self.captain_update_publisher(local_map_message)

        elif self.test_count == 6:
            self.get_logger().info(f"Sending beacon 5 reached message")

            local_map_message.data = "5 reached"

            self.captain_update_publisher(local_map_message)

        else:

            self.get_logger().info("TESTS PASSED SUCCESSFULLY")

            # Exit
            self.destroy_node()


# Initialize node
def main(args=None):
    rclpy.init()

    test_node = PlotNavigationNodeTest()

    # Run Node on interval specified in the __init__ function, until it is killed
    rclpy.spin(test_node)

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
