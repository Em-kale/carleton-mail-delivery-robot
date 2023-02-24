#!/usr/bin/env python
# @author: Stephen Wicklund, Jacob Charpentier, Favour Olotu
# SUBSCRIBER:   beacons
# SUBSCRIBER:   navigator
# PUBLISHER:    navigationMap
# PUBLISHER:    localMap
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
import os


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


def sign(x):
    return bool(x > 0) - bool(x < 0)


class JunctionSlopeTracker():
    def __init__(self, N):
        self.dataQueue = []
        self.averageQueue = []
        self.slopeQueue = []
        self.N = N
        self.sum = 0

        self.counter = 0
        self.signalSent = False

    def addDataPoint(self, dataPoint, logger):
        if (len(self.averageQueue) != 0 and abs(int(dataPoint) - int(self.averageQueue[-1])) > int(
                magicNumbers['BEACON_OUTLIER_THRESHOLD'])):
            return False

        # Remove data point exceeding window size N
        if len(self.dataQueue) >= self.N:
            self.sum -= int(self.dataQueue.pop(0))

        # Add new data point and update sum and average queue
        self.dataQueue.append(dataPoint)
        self.sum += int(dataPoint)
        self.averageQueue.append(self.sum / len(self.dataQueue))
        self.counter += 1
        logger.debug("Average: %s" % self.averageQueue[-1])
        # slope assume equal distance between points i.e divded by 1
        # Let 5 points accumulate, then take simple slope.
        if len(self.averageQueue) >= 3 and self.counter == 3:
            self.slopeQueue.append(self.averageQueue[-1] - self.averageQueue[-3])
            self.counter = 0
            logger.debug("Slope: %s" % self.slopeQueue[-1])

        # Slope change is confirmed if newest slope is not equal to last two slopes.
        # 3rd slope is popped
        if len(self.slopeQueue) >= 3:
            signChange = sign(self.slopeQueue[-1]) != sign(self.slopeQueue[-2]) and \
                         sign(self.slopeQueue[-1]) != sign(self.slopeQueue.pop(2))

            # Stop repeated signals
            return signChange
        else:
            return False


class Captain(Node):

    def __init__(self):
        super().__init__('captain')
        self.junctions = {}
        self.mapPublisher = self.create_publisher(String, 'navigationMap', 10)
        self.local_map_publisher = self.create_publisher(String, 'localMap', 10)
        self.beaconSubscriber = self.create_subscription(String, 'beacons', self.readBeacon, 10)
        self.global_map_subscriber = self.create_subscription(String, 'navigator', self.read_navigator_message, 10)
        self.direction = String()

    def passedBeacon(self, beacon_id):
        global_map_update = String()
        global_map_update.data = beacon_id + " Passed"
        self.local_map_publisher.publish(global_map_update)
        # tell robot driver to start moving in the defined direction
        self.mapPublisher.publish(self.direction)

    def reachedBeacon(self, beacon_id):
        pass

    def readBeacon(self, beacon):
        """
        Expected beacon data: "EE:16:86:9A:C2:A8,-40,4"
        """
        self.get_logger().debug('Received: "%s"' % beacon.data)

        if beacon.data.split(",")[0] in self.junctions:
            if self.junctions[beacon.data.split(",")[0]].addDataPoint(beacon.data.split(",")[1], self.get_logger()) \
                    and beacon.data.split(",")[1] <= magicNumbers.THRESHOLD_RSSI:
                self.get_logger().info('Passed beacon: "%s"' % beacon.data.split(",")[0])
                self.passedBeacon(beacon.data.split(",")[2])
            elif beacon.data.split(",")[1] >= magicNumbers.THRESHOLD_RSSI:
                self.get_logger().info('Reached beacon: "%s"' % beacon.data.split(",")[0])
                self.reachedBeacon(beacon.data.split(",")[2])
        else:
            self.junctions[beacon.data.split(",")[0]] = JunctionSlopeTracker(10)
            self.junctions[beacon.data.split(",")[0]].addDataPoint(beacon.data.split(",")[1], self.get_logger())

        """
        if False:
            f = open('captainLog.csv', "a")
            f.write(beacon.data + "\n")
            f.close()
        """

    def read_navigator_message(self, nav_message):
        """
        This function parses the message from the navigator and uses it to
        send the directional message to the robot driver node

        expected nav_message: "1 2 straight 4 Destination" => travel from junction 1 to 2 straight through beacon 4 Destination junction ID = 2
        expected nav_message: "1 2 straight 4" => travel from junction 1 to 2 straight through beacon 4
        expected nav_message: "Destination" implies the robot just passed the beacon for destination junction (Docking should be initiated)

        """
        self.get_logger().debug('Received: "%s"' % nav_message.data)

        message_list = nav_message.data.split(" ")
        reply_message = String()

        # if the robot as arrived at the destination send a docking message
        if len(message_list) == 1 and message_list[0] == "Destination":
            # TODO Need to confirm what message should be sent to the robot driver to initiate docking
            reply_message.data = "Dock"
            self.mapPublisher.publish(reply_message)

        # if the next junction is the destination send a destination approach message
        # elif len(message_list) == 5 and message_list[4] == "Destination":
            # TODO determine how that information would be useful to the robot driver

        elif message_list[2] != "straight":
            self.direction = message_list[2]
            reply_message.data = "turn on approach"
            self.mapPublisher.publish(reply_message)

        # by default the direction is straight send that to the robot driver
        else:
            self.direction = message_list[2]
            reply_message.data = message_list[2]
            self.mapPublisher.publish(reply_message)


DEBUG = False


def main():
    rclpy.init()
    captain = Captain()

    while (DEBUG):
        mapUpdate = String()
        mapUpdate.data = input("Enter a navigational update: ")
        captain.mapPublisher.publish(mapUpdate)
        captain.get_logger().debug('Publishing: "%s"' % mapUpdate.data)

    rclpy.spin(captain)


if __name__ == '__main__':
    main()
