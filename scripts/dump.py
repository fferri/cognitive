#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import sys
import thread
import rospy

from memory.lib import *

def dump():
    print "memory contents:"
    c = MemoryClient('cmdline', ns=rospy.get_param('~memory_namespace', '/memory'))
    metas = c.get_all()
    print_terms_table(metas)

if __name__ == "__main__":
    dump()
