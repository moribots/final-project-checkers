#!/usr/bin/env python
"""
A simple unit test to check if the Board_State topic is publishing and if it's
outputing 64 square states.
"""

import sys, unittest, time
import rospy, rostest
from std_msgs.msg import *

PKG = 'checkers'
NAME = 'bridge_unit_test'

class Test_Board_State_Listener(unittest.TestCase):
    def __init__(self, *args):
        super(TestTalkerListener, self).__init__(*args)
        self.success = False

    def callback(self, data):
        print(rospy.get_caller_id(), "I heard %s"%data.data)
        # Check if data is there and if it's outputing 64 square states
        if data.data and len(data.data.split(" ")) == 64:
            self.success = True

    def test_talker_listener(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.Subscriber("Board_State", String, self.callback)
        timeout_t = time.time() + 20.0 #20 seconds
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.success)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, Test_Board_State_Listener, sys.argv)
