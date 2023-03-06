#!/usr/bin/env python
# @author: Favour Olotu
import queue

# SUBSCRIBER:   localMap
# SUBSCRIBER:   Requests
# PUBLISHER:    navigator

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import navigation_utilities


class plot_navigation(Node):

    def __init__(self):
        super().__init__('plot_navigation')

        self.navigation_publisher = self.create_publisher(String, 'navigator', 10)
        self.request_subscriber = self.create_subscription(String, 'Requests', self.handle_request, 10)
        self.local_map_subscriber = self.create_subscription(String, 'localMap', self.parse_local_map_data, 10)

        self.map_graph = navigation_utilities.load_tunnel_map_graph('../map.csv')
        self.current_path = None
        self.beacon_to_pass = None
        self.navigation_queue = queue.Queue()
        self.previous_direction = None
        self.counter = 0

    def add_to_direction_queue(self, direction, beacon_id):
        """
        This method unpacks the directions from the navigational map
        append direction beaconId to the queue
        """
        if direction == "left":
            self.navigation_queue.put("straight " + beacon_id)
            self.navigation_queue.put("u-turn " + beacon_id)
            self.navigation_queue.put("right " + beacon_id)
        else:
            self.navigation_queue.put(direction + " " + beacon_id)

    def handle_request(self, request):
        """
        This function handles a request from the server with the information
        of the source and destination junction of the mail delivery route

        Assumption: The server jobs are always from the current location of the robot
        [To be updated as the functionality for job queuing is added]

        expected data: "1 12" - start point is junction 1 and destination is junction 12

        Navigation Message rubric: DirectionForGettingToNextJunction beaconIDForNextJunction [?Destination]
        Destination is only added if the next junction is the destination
        Sample message published: "Straight 4 Destination" => travel straight through beacon 4 which is the destination
        Sample message published: "Straight 4" => travel straight through beacon 4

        Note: Messages should be separated by a space
        """
        job = request.data.split(" ")
        self.get_logger().info(
            'Received request from server to travel from junction: ' + job[0] + 'to junction: ' + job[1])

        # TODO Logic for queuing and queueing request while a current path is being followed
        # Alternatively it my be worth it to implement on the server side
        self.current_path = navigation_utilities.breadth_first_search(self.map_graph, job[0], job[1])
        self.counter = 0
        self.get_logger().info("Current travel path: " + str(self.current_path))

        navigation_message = String()

        # If the current path only includes two junctions add destination flag to message
        if len(self.current_path) == 2:
            navigation_message = navigation_utilities.determine_next_direction(self.map_graph,
                                                                               navigation_utilities.expectedBeacon(
                                                                                   self.map_graph, self.current_path[0],
                                                                                   self.current_path[0])) \
                                 + " " + navigation_utilities.expectedBeacon(self.map_graph, self.current_path[0],
                                                                             self.current_path[0]) \
                                 + " " + "Destination"
            self.counter += 2

        elif len(self.current_path) == 1:
            navigation_message = "Destination"

        else:
            navigation_message = navigation_utilities.determine_next_direction(self.map_graph,
                                                                               navigation_utilities.expectedBeacon(
                                                                                   self.map_graph, self.current_path[0],
                                                                                   self.current_path[0])) \
                                 + " " + navigation_utilities.expectedBeacon(self.map_graph, self.current_path[0],
                                                                             self.current_path[0])
            self.counter += 1
        # initial message
        self.navigation_publisher.publish(navigation_message)

    def parse_local_map_data(self, local_map_update):
        """
                This function handles a local map update from the captain node with the information
                of the source and destination junction of the mail delivery route

                expected local map update: "4 Passed" - Beacon ID 4 has been passed by the robot

                Navigation Message rubric: currentJunctionID nextJunctionID DirectionForGettingToNextJunction beaconIDForNextJunction [Destination]
                Destination is only added if the next junction is the destination

                Sample message published: "1 2 straight 4 Destination" => travel from junction 1 to 2 straight through beacon 4 Destination junction ID = 2
                Sample message published: "1 2 straight 4" => travel from junction 1 to 2 straight through beacon 4
                Sample message published: "Destination" implies the robot just passed the beacon for destination junction (Docking should be initiated)

                Note: Messages should be separated by a space
        """
        map_update = local_map_update.split(" ")

        # Handle a beacon reached update
        if map_update == "Reached":
            # do something
            return

        # Handle beacon passed messages
        if len(self.current_path) == self.counter + 1 and map_update[0] == self.beacon_to_pass:
            navigation_message = "Destination"
            self.counter += 1

        elif len(self.current_path) == self.counter + 2 and map_update[0] == self.beacon_to_pass:
            navigation_message = self.current_path[0] + " " + self.current_path[1] + " " + \
                                 navigation_utilities.determine_next_direction(self.map_graph,
                                                                               navigation_utilities.expectedBeacon(
                                                                                   self.map_graph, self.current_path[0],
                                                                                   self.current_path[0])) \
                                 + " " + navigation_utilities.expectedBeacon(self.map_graph, self.current_path[0],
                                                                             self.current_path[0]) \
                                 + " " + "Destination"
            self.counter += 1

        elif map_update[0] == self.beacon_to_pass:
            navigation_message = self.current_path[0] + " " + self.current_path[1] + " " + \
                                 navigation_utilities.determine_next_direction(self.map_graph,
                                                                               navigation_utilities.expectedBeacon(
                                                                                   self.map_graph, self.current_path[0],
                                                                                   self.current_path[0])) \
                                 + " " + navigation_utilities.expectedBeacon(self.map_graph, self.current_path[0],
                                                                             self.current_path[0])
            self.counter += 1

        self.navigation_publisher.publish(navigation_message)


def main():
    rclpy.init()
    plot_navigator = plot_navigation()
    rclpy.spin(plot_navigator)


if __name__ == '__main__':
    main()
