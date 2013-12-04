#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import sys
import thread
import rospy

from memory.lib import *

if __name__ == "__main__":
    rospy.init_node('memory_remove')
    c = MemoryClient('cmdline', ns=rospy.get_param('~memory_namespace', '/memory'))
    argv = rospy.myargv()
    try:
        term_id = int(argv[1])
        metas = [c.remove_id(term_id)]
    except ValueError:
        term = term_parse(argv[1:])
        metas = c.remove(term)
    print "Removed %d terms." % len(metas)

