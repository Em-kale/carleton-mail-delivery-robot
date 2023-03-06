import time
import math
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import sys
from bluepy.btle import Scanner, DefaultDelegate
import csv
import os 
# Class instantiation which enables handleNotification and handleDiscovery debugging logs
class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

def loadBeaconData():
    """
    Loads all beacons into a list from beaconList.csv
    :return: List
    """
    beacons = {}
    ROOT_DIR = os.getcwd()
    with open(f'{ROOT_DIR}/src/carleton-mail-delivery-robot/mail_delivery_robot/beaconList.csv') as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        count = 0
        for row in reader:
            if count == 0:
                count += 1
                continue
            beacons[row[0]] = row[1]  # row[0] = ID, row[1] = MAC ADDR
    return beacons

DEBUG = False
class BeaconReader(Node):
    def __init__(self):
        super().__init__('beacon_reader')
        self.beacons = loadBeaconData()
        for beacon in self.beacons:
            self.get_logger().info("beacon data {}".format(beacon)) # Fill logger with each beacon data
        self.publisher_ = self.create_publisher(String, 'beacons', 10) # Create a publisher of String msgs named beacons
        timer_period = 0.5 # Seconds
        self.timer = self.create_timer(timer_period, self.checkForBeacons) # call checkForBeacons() every 0.5 seconds
        self.scanner = Scanner().withDelegate(ScanDelegate()) # Create Scanner Class
 

    def checkForBeacons(self):
        
        devices = self.scanner.scan(0.4) # Listen for ADV_IND packages for 0.4 seconds
        beaconData = String()
        
        # For each scanned device check if device address matches beacon in list
        for dev in devices:
            for beacon in self.beacons:
                if(self.beacons[beacon] == dev.addr):
                    # Log successful device detection and signal strength
                    self.get_logger().info("Device {} ({}), RSSI={} dB".format(dev.addr, dev.addrType, dev.rssi))
                    beaconData.data = beacon + "," + str(dev.rssi) # Configure message from beacon data
                    self.publisher_.publish(beaconData) # Publish Message
                    if True:
                        f = open('captainLog.csv', "a") # log to captainLog.csv file
                        f.write(beaconData.data +"\n")
                        f.close()



def main(): # instantiate everything
    rclpy.init()
    beacon_reader = BeaconReader()

    rclpy.spin(beacon_reader)
    


if __name__ == '__main__':
    main()
