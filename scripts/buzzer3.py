#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt16

def write_freq(hz=0):
    bfile = "/dev/rtbuzzer0"
    try:
        with open(bfile, "w") as f:
            f.write(str(hz) + "\n")
    except IOError:
        rospy.logerr("can't write to " + bfile)

def recv_buzzer(data):
    write_freq(data.data)

if __name__ == '__main__':
    # ノードの初期化、ROS に登録される
    rospy.init_node('buzzer')
    # (トピックの名前, トピックの型, コールバック関数): 外からメッセージが来たら、コールバック関数を起動する
    rospy.Subscriber("buzzer", UInt16, recv_buzzer)
    # ノードが終わらないようにする
    rospy.spin()
