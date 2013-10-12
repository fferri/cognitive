#!/usr/bin/env python

import roslib

roslib.load_manifest('memory')

import sys
import thread

import rospy

from std_msgs.msg import *
from memory.msg import *
from memory.srv import *
from common import *
from client import MemoryClient

if __name__ == "__main__":
    c = MemoryClient('cmdline')
    try:
        term_id = int(sys.argv[1])
        metas = [c.remove_id(term_id)]
    except ValueError:
        term = term_parse(sys.argv[1:])
        metas = c.remove(term)
    print "Removed %d terms." % len(metas)

