#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import sys
import thread
import rospy

from memory.lib import *

if __name__ == "__main__":
    rospy.init_node('memory_change')
    c = MemoryClient('cmdline', ns=rospy.get_param('~memory_namespace', '/memory'))
    argv = rospy.myargv()
    t = term_parse(argv[2:])
    meta = c.change(int(argv[1]), t)
    print "Changed (id=%d)." % meta.term_id
