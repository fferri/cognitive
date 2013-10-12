#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import roslib.message
import sys
import thread
import rospy

from memory.lib import *

def mapper():
    rospy.init_node('memory_mapper')
    topic_name = sys.argv[1]
    topic_type = sys.argv[2]
    timeout = int(sys.argv[3])
    term_functor = sys.argv[4]
    term_args_expr = sys.argv[5:]

    topic_class = roslib.message.get_message_class(topic_type)
    if not topic_class:
        print('Message %s not found. (are messages built?)' % topic_type)
        return

    mm = MemoryTopicMapper(topic_name, topic_class, timeout, term_functor, term_args_expr)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        mm.spin()
        r.sleep()

if __name__ == "__main__":
    mapper()

