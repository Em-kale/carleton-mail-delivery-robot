#!/usr/bin/env python
# @author: Stephen Wicklund

# SUBSCRIBER:   NONE
# PUBLISHER:    Requests
import rospy
from std_msgs.msg import String

def rosMain():
    rospy.init_node('receiveRequest', anonymous=True)
    # subscriber to navigation
    rate = rospy.Rate(5)
    
    while not rospy.is_shutdown():
        command = -1
        if command == -1:
            pass
        else:
            pass
        rate.sleep()

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass