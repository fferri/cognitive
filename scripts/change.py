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
    t = term_parse(sys.argv[2:])
    meta = c.change(int(sys.argv[1]), t)
    print "Changed (id=%d)." % meta.term_id
