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
from client import MemoryClient

def show_log():
    rospy.init_node('memory_log')
    c = MemoryClient('cmdline')

    log_size = c.log_size()

    changes = c.log_get(range_min=1, range_max=log_size)
    if changes == None:
        print("Server returned error")
        return

    print("Server returned %d entries" % len(changes))
    n = 1
    for change in changes:
        if change.op_type == Change.OP_ADD:
            op = 'added'
        elif change.op_type == Change.OP_DEL:
            op = 'removed'
        elif change.op_type == Change.OP_CHANGE:
            op = 'changed'
        else:
            raise Exception('Unknown op_type: %d' % change.op_type)
        e = '#%d: %d.%d: %s %s term %s (id=%d)' % (n, change.meta.stamp.secs, change.meta.stamp.nsecs, change.src, op, term_to_string(change.meta.term), change.meta.term_id)
        n += 1
        print(e)

if __name__ == "__main__":
    show_log()
