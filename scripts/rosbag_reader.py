#!/usr/bin/env python2
# -*- coding: <utf-8> -*-
import sys
from copy import deepcopy
import rosbag

bag = rosbag.Bag(sys.argv[1])

first_message1 = True
first_message2 = True

intial1 = None
intial2 = None

data1 = []
data2 = []

for topic1, msg1, t1 in bag.read_messages(topics=[sys.argv[2]]):
    time1 = msg1.header.stamp.secs * (10 ** 9) + msg1.header.stamp.nsecs
    if first_message1:
        initial1 = deepcopy(msg1.pose.position)
        first_message1 = False
    message2 = None
    for topic2, msg2, t2 in bag.read_messages(topics=[sys.argv[3]]):
        temp_time2 = msg2.header.stamp.secs * (10 ** 9) + msg2.header.stamp.nsecs
        if first_message2:
            initial2 = deepcopy(msg2.pose.position)
            first_message2 = False
        if temp_time2 < time1:
            message2 = deepcopy(msg2)
            data1.append([msg1.pose.position.x - initial1.x, msg1.pose.position.y - initial1.y, msg1.pose.position.z - initial1.z])
            data2.append([message2.pose.position.x - initial2.x, msg2.pose.position.y - initial2.y, msg2.pose.position.z - initial2.z])
        else:
            break
print(len(data1))
print(len(data2))
# for topic, msg, t in bag.read_messages(topics=[sys.argv[2], sys.argv[3]]):
#     print str(msg.pose.position) + "\t" + topic + "\t" + str(msg.header.stamp.secs)
bag.close()
