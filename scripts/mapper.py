#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import roslib.message
import sys
import thread
import rospy

from memory.lib import *

def mapper():
    rospy.init_node('memory_mapper')
    namespace = rospy.get_param('~memory_namespace', '/memory')
    topic_name = rospy.get_param('~topic_name')
    topic_type = rospy.get_param('~topic_type')
    timeout = int(rospy.get_param('~timeout', '0'))
    term_functor = rospy.get_param('~term/functor')
    term_args_expr = []
    i = 0
    while rospy.has_param('~term/args/%d' % i):
        term_args_expr.append(rospy.get_param('~term/args/%d' % i))
        i += 1

    topic_class = roslib.message.get_message_class(topic_type)
    if not topic_class:
        print('Message %s not found. (are messages built?)' % topic_type)
        return

    mm = MemoryTopicMapper(topic_name, topic_class, term_functor, term_args_expr, timeout=timeout, ns=namespace)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        mm.spin()
        r.sleep()

if __name__ == "__main__":
    mapper()

