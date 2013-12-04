#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import sys
import thread
import rospy

from memory.lib import *

if __name__ == "__main__":
    rospy.init_node('memory_add')
    c = MemoryClient('cmdline', ns=rospy.get_param('~memory_namespace', '/memory'))
    argv = rospy.myargv()
    t = term_parse(argv[1:])
    meta = c.add(t)
    print "Added (id=%d)." % meta.term_id
