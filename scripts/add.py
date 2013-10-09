#!/usr/bin/env python

import roslib

roslib.load_manifest('memory')

import sys
import thread

import rospy

from std_msgs.msg import *
from memory.msg import *
from memory.srv import *

if __name__ == "__main__":
    t = Term()
    t.functor = sys.argv[1]
    t.args = []
    for arg in sys.argv[2:]:
        a = Atom()
        try:
            a.intData = [int(arg)]
        except:
            try:
                a.floatData = [float(arg)]
            except:
                a.stringData = [arg]
        t.args.append(a)
    memory_srv_add = rospy.ServiceProxy('/memory/add', AddTerm)
    resp = memory_srv_add(AddTermRequest(source='cmdline', term=t))
    if resp.succeeded:
        print "Succeeded."
    else:
        print "Failed."

