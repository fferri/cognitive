#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import sys
import thread
import rospy

from memory.lib import *
from rostopic import get_topic_class

if __name__ == "__main__":
    rospy.init_node('memory_mapper')
    mm = MemoryTopicMapper(sys.argv[1], get_topic_class(sys.argv[2]), sys.argv[3], sys.argv[4:])
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        r.sleep()

