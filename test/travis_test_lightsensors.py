#!/usr/bin/env python
# -*- coding: utf-8 -*-
import unittest, rostest
import rosnode, rospy
import time
from pimouse_ros.msg import LightSensorValues

class LightsensorTest(unittest.TestCase):
    def setUp(self):
        self.count = 0
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)
        self.values = LightSensorValues()

    def callback(self, data):
        self.count += 1
        self.values = data

    def check_values(self, lf, ls, rs, rf):
        vs = self.values
        self.assertEqual(vs.left_forward,     lf, "different values: left_forward")
        self.assertEqual(vs.left_side,        ls, "different values: left_side")
        self.assertEqual(vs.right_side,       rs, "different values: right_side")
        self.assertEqual(vs.left_forward,     rf, "different values: left_forward")
        self.assertEqual(vs.sum_all, lf+ls+rs+rf, "different values: sum_all")
        self.assertEqual(vs.sum_forward,   lf+rf, "different values: sum_forward")

    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/lightsensors', nodes, "node does not exist")

    lf = 4321
    ls = 123
    rs = 0
    rf = -1

    def test_get_value(self):
        rospy.set_param('lightsensors_freq', 10)
        time.sleep(2)
        with open("/dev/rtlightsensor0", 'w') as f:
            f.write("%d %d %d %d" % (self.rf, self.rs, self.ls, self.lf))

        time.sleep(3)

        self.assertFalse(self.count == 0, "cannot subscribe the topic")
        self.check_values(self.lf, self.ls, self.rs, self.rf)

    def test_change_parameter(self):
        rospy.set_param('lightsensors_freq', 1)
        time.sleep(2)
        c_prev = self.count
        time.sleep(3)
        self.assertTrue(self.count < c_prev + 4, "freq does not change")
        self.assertFalse(self.count == c_prev, "subscriber is stopped")


if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('travis_test_lightsensors')
    rostest.rosrun('pimouse_ros', 'travis_test_lightsensors', LightsensorTest)
