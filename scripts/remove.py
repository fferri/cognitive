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
    id = int(sys.argv[1])
    memory_srv_remove = rospy.ServiceProxy('/memory/remove', RemoveTerm)
    resp = memory_srv_remove(RemoveTermRequest(source='cmdline', id=id))
    if resp.succeeded:
        print "Succeeded."
    else:
        print "Failed."

