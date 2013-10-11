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

def show_log():
    rospy.init_node('memory_log')

    print("Waiting for services coming up...")
    rospy.wait_for_service('/memory/poll')
    rospy.wait_for_service('/memory/get')

    print("Polling for log size...")
    memory_srv_poll = rospy.ServiceProxy('/memory/poll', Poll)
    try:
        resp = memory_srv_poll(PollRequest())
        print("Log size is %d" % resp.logsize)
        logmin = 1
        logmax = resp.logsize
    except rospy.ServiceException as exc:
        print("Server did not process request: " + str(exc))
        return

    print("Getting log %d-%d..." % (logmin, logmax))
    memory_srv_get = rospy.ServiceProxy('/memory/get', Get)
    try:
        resp = memory_srv_get(GetRequest(range_min=logmin, range_max=logmax))
        n = resp.range_min
        if not resp.succeeded:
            print("Server returned error")
            return
        print("Server returned %d entries" % len(resp.changes))
    except rospy.ServiceException as exc:
        print("Server did not process request: " + str(exc))
        return
    
    for change in resp.changes:
        op = 'removed' if change.removedTermId > 0 else 'added'
        e = '#%d: %d.%d: %s %s term %s (id=%d)' % (n, change.stamp.secs, change.stamp.nsecs, change.source, op, term_to_string(change.addedOrRemovedTerm), change.addedOrRemovedTerm.term_id)
        n += 1
        print(e)

if __name__ == "__main__":
    show_log()
