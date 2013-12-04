#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import sys
import thread
import rospy

from memory.lib import *

if __name__ == "__main__":
    rospy.init_node('memory_clear')
    c = MemoryClient('cmdline', ns=rospy.get_param('~memory_namespace', '/memory'))
    metas = c.get_all()
    numRemoved = 0
    for meta in metas:
        print('Removing term #%d...' % meta.term_id)
        c.remove_id(meta.term_id)
        numRemoved += 1
    print('Removed %d terms out of %d.' % (numRemoved, len(metas)))

