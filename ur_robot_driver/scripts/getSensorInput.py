#!/usr/bin/env python

import sys
import time

from ur_msgs.srv import *
from ur_msgs.msg import IOStates
from std_msgs.msg import String

import rospy

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input


class SensorGetter:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self, service = '/ur_hardware_interface/io_states'):
        
        #print('Getting subscriber')
        #rospy.wait_for_service(service)
        #print("\n\n\ninit\n\n\n")
        #self.subscriber = rospy.Subscriber(service, IOStates, self.callback)
        self.buttonState = False
        rospy.Subscriber(service, IOStates, self.callback)
        #rospy.spin()


    def callback(self, data):
        state = data.digital_in_states[0].state
        #rospy.loginfo(state)
        self.buttonState = state

    #def listener(self):

    def getSensorState(self):
        #time.sleep(0.005)
        return self.buttonState

#if __name__ == "__main__":
    #message = rospy.wait_for_message('/ur_hardware_interface/io_states', IOStates)
    #print(message)
    #for i in range(0, 10000):
        #print(client.getSensorState())
