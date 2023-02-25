#!/usr/bin/env python
# @author: Favour Olotu

import csv
from typing import Union, Any


def load_tunnel_map_graph(map_filename: str) -> list:
    """
    This function creates a graph representation of the map of the tunnel
    from the map.csv file
    Returns the graph representation in a list called map_graph
    """
    map_graph = []
    with open(map_filename) as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        row_count = 0
        for row in reader:
            if row_count == 0:
                row_count += 1
                continue
            map_graph.append((row[0], ((row[1], row[2]), (row[3], row[4]), (row[5], row[6]), (row[7], row[8]))))
    return map_graph


def junction_id_to_vertex_number(map_graph: list, junction_id: str) -> int:
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


def breadth_first_search(map_graph: list, source_junction: str, destination_junction: str) -> list:
    """
    This function performs a breadth first search on the map graph
    to find the shortest path from the source to the destination junction.
    Returns a list of junctionIDs of shortest path
    """
    root = junction_id_to_vertex_number(map_graph, source_junction)

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
            num = junction_id_to_vertex_number(map_graph, i[1])
            if (num != -1 and visited[num] == False):
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


def beacon_to_junction(map_graph: list, beacon_id: str) -> Union[int, Any]:
    """
    This function iterates through the graph of the map and returns
    the junction id associated with the given beacon id
    or -1 if beacon id not found
    """
    for junction in map_graph:
        for beacon in junction[1]:
            if beacon[0] == beacon_id:
                return junction[0]
    return -1


def expectedBeacon(map_graph: list, source_junction: str, destination_junction: str) -> Union[int, Any]:
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


def determine_next_direction(map_graph: list, beacon_id: str, junction_id: str) -> str:
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


# Example
if __name__ == "__main__":
    mapGraph = load_tunnel_map_graph('../map.csv')
    path = breadth_first_search(mapGraph, "1", "5")
    print("Path going from 1 to 13: " + str(path))
    print("Starting going straight South from 1")



    count = 0
    print("At beacon " + str(expectedBeacon(mapGraph, path[0], path[1]))
        + ", junction " + str(beacon_to_junction(mapGraph, expectedBeacon(mapGraph, path[0], path[1]))))

    print("\t\t    ------> " + determine_next_direction(mapGraph, expectedBeacon(mapGraph, path[0], path[1]),
                                                        path[2])
          + " to junction " + path[2])

    print ("----------------------------------------------------------------------------------------------------")
    for junction in path:
        print("At beacon " + str(expectedBeacon(mapGraph, junction, path[count + 1]))
          + ", junction " + str(beacon_to_junction(mapGraph, expectedBeacon(mapGraph, junction, path[count + 1]))))
        if (count + 2 == len(path)):
            print("Arrived at destination!")
            break
        print("\t\t    ------> " + determine_next_direction(mapGraph, expectedBeacon(mapGraph, junction, path[count + 1]),
                                                        path[count + 2])
          + " to junction " + path[count + 2])
        count += 1
