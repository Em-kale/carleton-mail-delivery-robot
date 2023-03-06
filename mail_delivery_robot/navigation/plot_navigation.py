#!/usr/bin/env python
# @author: Favour Olotu
import queue
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
import os


# SUBSCRIBER:   localMap
# SUBSCRIBER:   Requests
# PUBLISHER:    navigator


def load_tunnel_map_graph() -> list:
    """
    This function creates a graph representation of the map of the tunnel
    from the map.csv file
    Returns the graph representation in a list called map_graph
    """
    ROOT_DIR = os.getcwd()
    map_graph = []
    with open(f'{ROOT_DIR}/src/carleton-mail-delivery-robot/mail_delivery_robot/development_map.csv') as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        row_count = 0
        for row in reader:
            if row_count == 0:
                row_count += 1
                continue
            map_graph.append((row[0], ((row[1], row[2]), (row[3], row[4]), (row[5], row[6]), (row[7], row[8]))))
    return map_graph


class NavigationUtilities:

    def junction_id_to_vertex_number(self, map_graph: list, junction_id: str) -> int:
        """
        This function maps junction id array to numerically ordered array
        i.e. junctions ["1","5","8"] maps to [0,1,2]
        """
        index = 0
        for vertex in map_graph:
            if vertex[0] == junction_id:
                return index
            index += 1
        return -1

    def breadth_first_search(self, map_graph: list, source_junction: str, destination_junction: str) -> list:
        """
        This function performs a breadth first search on the map graph
        to find the shortest path from the source to the destination junction.
        Returns a list of junctionIDs of shortest path
        """
        root = self.junction_id_to_vertex_number(map_graph, source_junction)

        visited = [False] * len(map_graph)
        queue = []
        traceback = []
        traversal = []
        found = False

        queue.append(root)
        visited[root] = True

        while queue:
            root = queue.pop(0)
            traversal.append(root)

            for i in map_graph[root][1]:
                if i[1] == destination_junction:
                    traceback.append(i[1])
                    found = True
                    break
                num = self.junction_id_to_vertex_number(map_graph, i[1])
                if num != -1 and visited[num] == False:
                    queue.append(num)
                    visited[num] = True

            if found:
                traversal.reverse()
                for vertex in traversal:
                    for i in map_graph[vertex][1]:
                        if i[1] == traceback[-1]:
                            traceback.append(map_graph[vertex][0])
                            break
                traceback.reverse()
                return traceback

    def beacon_to_junction(self, map_graph: list, beacon_id: str):
        """
        This function iterates through the graph of the map and returns
        the junction id associated with the given beacon id
        or -1 if beacon id not found
        """
        for junction in map_graph:
            for beacon in junction[1]:
                if beacon[0] == beacon_id:
                    return beacon[1]
        return -1

    def expectedBeacon(self, map_graph: list, source_junction: str, destination_junction: str):
        """
        For a pair of source and destination junction in the map graph
        this function returns the expected beacon to be encountered
        """
        for junction in map_graph:
            if junction[0] == destination_junction:
                for beacon in junction[1]:
                    if beacon[1] == source_junction:
                        return beacon[0]
        return -1

    def determine_next_beacon(self, map_graph: list, source_junction: str, destination_junction: str):
        """
        For a pair of source and destination junction in the map graph
        this function returns the next beacon to be encountered
        """
        for junction in map_graph:
            if junction[0] == source_junction:
                for beacon in junction[1]:
                    if beacon[1] == destination_junction:
                        return beacon[0]
        return -1

    def determine_next_direction(self, map_graph: list, beacon_id: str, junction_id: str) -> str:
        """
        This function determines the desired direction to take based on the placement of the
        beacon and the desired junction id in the graph map
        """
        turns = [
            ["u-turn", "left", "straight", "right"],
            ["right", "u-turn", "left", "straight"],
            ["straight", "right", "u-turn", "left"],
            ["left", "straight", "right", "u-turn"],
        ]
        source_direction = None
        destination_direction = None
        for junction in map_graph:
            count = 0

            for beacon in junction[1]:
                if beacon[0] == beacon_id:
                    source_direction = count
                if beacon[1] == junction_id:
                    destination_direction = count
                count += 1

            if source_direction is not None and destination_direction is not None:
                break

        return turns[source_direction][destination_direction]


class plot_navigation(Node):

    def __init__(self):
        super().__init__('plot_navigation')

        self.captain_update_publisher = self.create_publisher(String, 'navigator', 10)
        self.request_subscriber = self.create_subscription(String, 'Requests', self.handle_request, 10)
        self.captain_update_subscriber = self.create_subscription(String, 'localMap', self.parse_local_map_data, 10)
        self.navigation_utilities = NavigationUtilities()
        self.map_graph = load_tunnel_map_graph()
        self.navigation_queue = None
        self.current_junction = None
        self.destination_junction = None

    def populate_navigation_queue(self, source_id, destination_id):
        """
        This method populates every navigation message required to move from one junction to another

        Navigation Message rubric: beaconIDForNextJunction directionAtBeacon
        Destination is only added if the next junction is the destination
        Sample message published: "4 Destination" => beacon 4 is the destination
        Sample message published: "4 Straight" => travel straight through beacon 4

        Note: Messages should be separated by a space

        """
        self.navigation_queue = queue.Queue()
        current_path = self.navigation_utilities.breadth_first_search(self.map_graph, source_id, destination_id)
        self.get_logger().info("Current travel path: " + str(current_path))

        self.navigation_queue.put(
            "initial " + self.navigation_utilities.determine_next_direction(self.map_graph,
                                                                            self.navigation_utilities.expectedBeacon(
                                                                                self.map_graph,
                                                                                current_path[0],
                                                                                current_path[1]),
                                                                            current_path[1]))

        count = 0

        for junction in current_path:

            beacon = str(
                self.navigation_utilities.determine_next_beacon(self.map_graph, junction, current_path[count + 1]))

            if count + 2 == len(current_path):
                self.navigation_queue.put(beacon + " destination")
                break

            direction = self.navigation_utilities.determine_next_direction(self.map_graph,
                                                                           self.navigation_utilities.expectedBeacon(
                                                                               self.map_graph, junction,
                                                                               current_path[count + 1]),
                                                                           current_path[count + 2])

            if direction == "left":
                self.navigation_queue.put(beacon + " straight")
                self.navigation_queue.put(beacon + " u-turn")
                self.navigation_queue.put(beacon + " right")
            else:
                self.navigation_queue.put(beacon + " " + direction)

            count += 1

        # initial message
        message = String()
        message.data = self.navigation_queue.get()
        self.captain_update_publisher.publish(message)

    def handle_request(self, request):
        """
        This function handles a request from the server with the information
        of the source and destination junction of the mail delivery route

        Assumption: The server jobs are always from the current location of the robot
        [To be updated as the functionality for job queuing is added]

        expected data: "1 12" - start point is junction 1 and destination is junction 12

        """

        job = request.data.split(" ")
        self.get_logger().info(
            'Received request from server to travel from junction: ' + job[0] + ' to junction: ' + job[1])

        # TODO Logic for queuing and queueing request while a current path is being followed
        # Alternatively it may be worth it to implement on the server side

        self.populate_navigation_queue(job[0], job[1])

    def parse_local_map_data(self, captain_update):
        """
                This function handles a local map update from the captain node
                It sends the next direction if the beacon ID matches

                expected local map update: "4 Passed" or "4 Reached"- Beacon ID 4 has been passed by the robot

                Navigation Message rubric: beaconID DirectionInstruction

                Sample message published: "4 straight" => travel straight through beacon 4
                Sample message published: "4 Destination" implies that beacon 4 is the destination(Docking should be initiated when reached)

                Note: Messages should be separated by a space
        """
        map_update = captain_update.data.split(" ")
        current_navigation = str(self.navigation_queue.get())
        direction = current_navigation.split(" ")

        # Check the beacon matches expected
        if map_update[0] == direction[0]:
            message = String()
            message.data = current_navigation
            self.captain_update_publisher.publish(message)

        # If the current beacon doesn't match the expected re-calculate the route
        else:
            # Determine current junction based on beacon
            self.navigation_utilities.navigation_utilities.beacon_to_junction(self.map_graph, map_update[0])

            # refresh navigation_queue
            self.populate_navigation_queue(self.current_junction, self.destination_junction)
            return

        # self.captain_update_publisher.publish(self.navigation_queue.get())


def main():
    rclpy.init()
    plot_navigator = plot_navigation()
    rclpy.spin(plot_navigator)


if __name__ == '__main__':
    main()
