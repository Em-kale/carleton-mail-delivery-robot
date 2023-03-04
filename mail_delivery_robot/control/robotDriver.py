#!/usr/bin/env python
# @author: Stephen Wicklund, Jacob Charpentier
# SUBSCRIBER:   preceptions
# PUBLISHER:    actions
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import csv
import os

# ~~~~ DEBUG MODE ~~~~
DEBUG_MODE = False


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


class DriverStateMachine:
    def __init__(self, initialState):
        self.currentState = initialState

    def handleNewDistanceEvent(self, data, actionPublisher):
        self.currentState = self.currentState.handleNewDistanceEvent(data, actionPublisher)

    def handleLostWallEvent(self, data, actionPublisher):
        self.currentState = self.currentState.handleLostWallEvent(data, actionPublisher)

    def handleCollisionEvent(self, data, actionPublisher):
        self.currentState = self.currentState.handleCollisionEvent(data, actionPublisher)

    def handleDockEvent(self, data, actionPublisher):
        self.currentState = self.currentState.handleDockEvent(data, actionPublisher)

    def handleReachedEvent(self, data, actionPublisher):
        self.currentState = self.currentState.handleReachedEvent(data, actionPublisher)


class DriverState:
    def handleNewDistanceEvent(self, data, actionPublisher):
        assert 0, "Must be implemented"

    def handleLostWallEvent(self, data, actionPublisher):
        assert 0, "Must be implemented"

    def handleCollisionEvent(self, data, actionPublisher):
        assert 0, "Must be implemented"

    def handleDockEvent(self, data, actionPublisher):
        assert 0, "Must be implemented"

    def handleReachedEvent(self, data, actionPublisher):
        assert 0, "Must be implemented"

    def toString(self):
        return ""


class FindWall(DriverState):

    def handleNewDistanceEvent(self, data, actionPublisher):
        # found wall so change state
        # TODO this should check that a wall is actually found before switching state.
        return WallFollowing()

    def handleLostWallEvent(self, data, actionPublisher):
        return self

    def handleCollisionEvent(self, data, actionPublisher):
        return CollisionHandling()

    def handleDockEvent(self, data, actionPublisher):
        return self

    def handleReachedEvent(self, data, actionPublisher):
        return self

    def toString(self):
        return "FindWall"


class WallFollowing(DriverState):

    def handleNewDistanceEvent(self, data, actionPublisher):
        action = String()
        action.data = data
        actionPublisher.publish(action)
        return self

    def handleLostWallEvent(self, data, actionPublisher):
        return FindWall()

    def handleCollisionEvent(self, data, actionPublisher):
        return CollisionHandling()

    def handleDockEvent(self, data, actionPublisher):
        return Docked()

    def handleReachedEvent(self, data, actionPublisher):
        if data.data == 'straight':
            return IntersectionReachedStraight()
        elif data.data == 'right':
            return IntersectionReachedRight()
        else:
            action = String()
            turn_radps = int(-math.pi / magicNumbers.TIMER_PERIOD)
            action.data = f"0,{turn_radps}"  # UTurn pi (left)
            actionPublisher.publish(action)
            return UTurn()

    def toString(self):
        return "WallFollowing"


class CollisionHandling(DriverState):

    def handleNewDistanceEvent(self, data, actionPublisher):
        return self

    def handleLostWallEvent(self, data, actionPublisher):
        return self

    def handleCollisionEvent(self, data, actionPublisher):
        # TODO: Add default collision handling
        return FindWall()

    def handleDockEvent(self, data, actionPublisher):
        return self

    def handleReachedEvent(self, data, actionPublisher):
        return self

    def toString(self):
        return "CollisionHandling"


class Docked(DriverState):

    def handleNewDistanceEvent(self, data, actionPublisher):
        return self

    def handleLostWallEvent(self, data, actionPublisher):
        return self

    def handleCollisionEvent(self, data, actionPublisher):
        return self

    def handleDockEvent(self, data, actionPublisher):
        return self

    def handleReachedEvent(self, data, actionPublisher):
        return self

    def toString(self):
        return "Docked"


class IntersectionReachedStraight(DriverState):

    def handleNewDistanceEvent(self, data, actionPublisher):
        action = String()
        action.data = data
        actionPublisher.publish(action)
        return self

    def handleLostWallEvent(self, data, actionPublisher):
        action = String()
        action.data = "0,0"  # straight
        actionPublisher.publish(action)
        return StraightTurn()

    def handleCollisionEvent(self, data, actionPublisher):
        return self

    def handleDockEvent(self, data, actionPublisher):
        return self

    def handleReachedEvent(self, data, actionPublisher):
        return self

    def toString(self):
        return "IntersectionReachedStraight"


class IntersectionReachedRight(DriverState):

    def handleNewDistanceEvent(self, data, actionPublisher):
        action = String()
        action.data = data
        actionPublisher.publish(action)
        return self

    def handleLostWallEvent(self, data, actionPublisher):
        action = String()
        turn_radps = int((math.pi / 2) / magicNumbers.TIMER_PERIOD)
        action.data = f"0,{turn_radps}"  # right turn pi/2
        actionPublisher.publish(action)
        return RightTurn()

    def handleCollisionEvent(self, data, actionPublisher):
        return self

    def handleDockEvent(self, data, actionPublisher):
        return self

    def handleReachedEvent(self, data, actionPublisher):
        return self

    def toString(self):
        return "IntersectionReachedRight"


class RightTurn(DriverState):

    def handleNewDistanceEvent(self, data, actionPublisher):
        return FindWall()

    def handleLostWallEvent(self, data, actionPublisher):
        return self

    def handleCollisionEvent(self, data, actionPublisher):
        return self

    def handleDockEvent(self, data, actionPublisher):
        return self

    def handleReachedEvent(self, data, actionPublisher):
        return self

    def toString(self):
        return "RightTurn"


class StraightTurn(DriverState):

    def handleNewDistanceEvent(self, data, actionPublisher):
        return FindWall()

    def handleLostWallEvent(self, data, actionPublisher):
        return self

    def handleCollisionEvent(self, data, actionPublisher):
        return self

    def handleDockEvent(self, data, actionPublisher):
        return self

    def handleReachedEvent(self, data, actionPublisher):
        return self

    def toString(self):
        return "StraightTurn"


class UTurn(DriverState):

    def handleNewDistanceEvent(self, data, actionPublisher):
        return FindWall()

    def handleLostWallEvent(self, data, actionPublisher):
        return self

    def handleCollisionEvent(self, data, actionPublisher):
        return self

    def handleDockEvent(self, data, actionPublisher):
        return self

    def handleReachedEvent(self, data, actionPublisher):
        return self

    def toString(self):
        return "UTurn"


class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        # Used for DEBUG only
        self.distance = 0.0
        self.angle = 0.0

        # configure publisher and subscribers
        self.actionPublisher = self.create_publisher(String, 'actions', 2)
        self.IRSubscriber = self.create_subscription(String, 'preceptions', self.updateIRSensor, 10)
        self.bumperEventSubscriber = self.create_subscription(String, 'bumpEvent', self.updateBumperState, 10)
        self.dockEventSubscriber = self.create_subscription(String, 'dockEvent', self.updateDockState, 10)
        # TODO These will be implemented in future commits
        self.reachedSubscriber = self.create_subscription(String, 'navigationMap', self.updateReachedState, 10)
        # initialize first state
        self.driverStateMachine = DriverStateMachine(FindWall())

    # update the robots distance flags based on data recieved from the IR sensors
    def updateIRSensor(self, data):
        # verify data type
        if (data.data != "-1"):  # TODO: -1 = Lost Wall
            # self.distance = data.data.split(",")[0]
            # self.angle = data.data.split(",")[1]

            # Make sure to pass it through the decode function first. actionTranslater will have to change in order to be able to handle this new message.
            # this is a String in the form 'target_distance:current_distance:current_angle'
            self.driverStateMachine.handleNewDistanceEvent(data, self.actionPublisher)
        else:
            self.driverStateMachine.handleLostWallEvent(data, self.actionPublisher)

        if (DEBUG_MODE):
            self.get_logger().info("Distance: " + str(self.distance) + "Angle: " + str(self.angle))

    def updateBumperState(self, data):
        # self.driverStateMachine.next(self.distanceFlags, self.bumperState)
        if (data.data != "unpressed"):
            self.driverStateMachine.handleCollisionEvent(data, self.actionPublisher)

        if (DEBUG_MODE):
            self.get_logger().debug("Bumper State: " + data.data)

    def updateDockState(self, data):
        # 3 Types of data "Docked", "Undocked", "StationFound"
        if (data.data != "Undocked"):
            self.driverStateMachine.handleDockEvent(data, self.actionPublisher)

        if (DEBUG_MODE):
            self.get_logger().debug("Dock State: " + data.data)

    def updateReachedState(self, data):
        self.driverStateMachine.handleReachedEvent(data, self.actionPublisher)

        if (DEBUG_MODE):
            self.get_logger().debug("Reached State: " + data.data)


def main():
    rclpy.init()
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)


if __name__ == '__main__':
    main()
