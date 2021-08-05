#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt16

def recv_buzzer(data):
    rospy.loginfo(type(data))
    rospy.loginfo(data.data)

if __name__ == '__main__':
    # ノードの初期化、ROS に登録される
    rospy.init_node('buzzer')
    # (トピックの名前, トピックの型, コールバック関数): 外からメッセージが来たら、コールバック関数を起動する
    rospy.Subscriber("buzzer", UInt16, recv_buzzer)
    # ノードが終わらないようにする
    rospy.spin()
