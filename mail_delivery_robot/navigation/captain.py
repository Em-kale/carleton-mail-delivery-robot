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
        
        for item in magicNumbers.items(): 
            print(item)

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
        self.slope = 0

        self.counter = 0
        self.signalSent = False

    def addDataPoint(self, dataPoint):
        if (len(self.averageQueue) != 0 and abs(int(dataPoint) - int(self.averageQueue[-1])) > int(
                magicNumbers['BEACON_OUTLIER_THRESHOLD'])):
            return False

        # Remove data point exceeding window size N
        if len(self.dataQueue) >= self.N:
            self.dataQueue.pop(0)

        # Add new data point and update sum and average queue
        self.dataQueue.append(int(dataPoint))
        summation = sum(self.dataQueue) 
        self.averageQueue.append(summation / len(self.dataQueue))
        
        self.counter += 1
        
        # slope assume equal distance between points i.e divded by 1
        # Let 5 points accumulate, then take simple slope.
        if len(self.averageQueue) >= 3 and self.counter == 3:
            self.slopeQueue.append(self.averageQueue[-1] - self.averageQueue[-3])
            self.counter = 0

            self.slope = self.averageQueue[-1] - self.averageQueue[-3]
        
        return sign(self.slope)

        # Slope change is confirmed if newest slope is not equal to last two slopes.
        # 3rd slope is popped
        # if len(self.slopeQueue) >= 3:
        #     signChange = sign(self.slopeQueue[-1]) != sign(self.slopeQueue[-2]) and \
        #                  sign(self.slopeQueue[-1]) != sign(self.slopeQueue.pop(2))
        #
        #     # Stop repeated signals
        #     return signChange
        # else:
        #     return False


class Captain(Node):

    def __init__(self):
        super().__init__('captain')
        self.beacon_subscriber = self.create_subscription(String, 'beacons', self.readBeacon, 10)
        self.robot_driver_publisher = self.create_publisher(String, 'navigationMap', 10)
        self.junctions = {}

    def passedBeacon(self, beacon_id):
        #determine which type of passed
        pass 

    def reachedBeacon(self, beacon_id):
        data = String()
        
        destination = 'MC' 
        direction = 'left' 

        if(beacon_id == 2 and destination == 'MC'):
            direction = 'left'
        elif(beacon_id == 2 and destination == 'ME'):
            direction = 'right'
        elif(beacon_id == 2 and destination == 'LA'):
            direction = 'straight'
        
        data.data = direction


        self.robot_driver_publisher.publish(data) 

    def readBeacon(self, beacon):
        """
        Expected beacon data: "EE:16:86:9A:C2:A8,-40,4"
        """
        self.get_logger().info('Received Beacon At Captain: "%s"' % beacon.data)
        #returns tuple (id, RSSI)
        beacon_id, beacon_rssi = beacon.data.split(",")
        threshold = magicNumbers['THRESHOLD_RSSI']
        
        if beacon_id in self.junctions:
            if self.junctions[beacon_id].addDataPoint(beacon_rssi) and beacon_rssi <= threshold:
                self.passedBeacon(beacon_id)
            elif self.junctions[beacon_id].addDataPoint(beacon_rssi) and beacon_rssi >= threshold:
                self.reachedBeacon(beacon_id)
        else:
            self.junctions[beacon_id] = JunctionSlopeTracker(10)
            self.junctions[beacon_id].addDataPoint(beacon_rssi)

    def read_navigator_message(self, nav_message):
        """
        This function parses the message from the navigator and uses it to
        send the directional message to the robot driver node
        expected nav_message: "4 Destination" => beacon 4 is the Destination (Initiate docking)
        expected nav_message: "4 straight" => travel straight through beacon 4
        expected directional message: "straight", "right", "u-turn", "dock"
        """
        self.get_logger().debug('Received: "%s"' % nav_message.data)

        message_list = nav_message.data.split(" ")
        reply_message = String()

        # if the robot as arrived at the destination send a docking message
        if message_list[1] == "destination":
            reply_message.data = "dock"
            self.robot_driver_publisher.publish(reply_message)

        # by default the direction is straight send that to the robot driver
        else:
            reply_message.data = message_list[1]
            self.robot_driver_publisher.publish(reply_message)


DEBUG = False


def main():
    rclpy.init()
    captain = Captain()

    while (DEBUG):
        mapUpdate = String()
        mapUpdate.data = input("Enter a navigational update: ")
        captain.robot_driver_publisher.publish(mapUpdate)
        captain.get_logger().debug('Publishing: "%s"' % mapUpdate.data)

    rclpy.spin(captain)


if __name__ == '__main__':
    main()
