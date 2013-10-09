#!/usr/bin/env python

import roslib

roslib.load_manifest('memory')

import sys
import thread

import rospy

from common import *

from std_msgs.msg import *
from memory.msg import *
from memory.srv import *

def dump():
    memory_srv_dump = rospy.ServiceProxy('/memory/dump', Dump)
    resp = memory_srv_dump(DumpRequest())
    for term in resp.terms:
        print "#%.03d %s added by '%s'" % (term.id, term_to_string(term), term.creator)
    print "log size: %d"%resp.logsize

if __name__ == "__main__":
    dump()
