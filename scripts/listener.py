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

class MemoryListener:
    def __init__(self, add_callback, remove_callback):
        self.mem = {}
        self.add_callback = add_callback
        self.remove_callback = remove_callback
        # get current memory snapshot
        memory_srv_dump = rospy.ServiceProxy('/memory/dump', Dump)
        resp = memory_srv_dump(DumpRequest())
        for term in resp.terms:
            self.mem[term.id] = term
        self.last_seq = resp.logsize
        rospy.Subscriber('/memory/seq', UInt32, self.callback_seq)

    def callback_seq(self, msg):
        if msg.data > self.last_seq:
            try:
                # get all edits
                memory_srv_get = rospy.ServiceProxy('/memory/get', Get)
                resp = memory_srv_get(GetRequest(range_min=self.last_seq+1, range_max=msg.data))
                if resp.succeeded:
                    for change in resp.changes:
                        term = change.addedOrRemovedTerm
                        # notify the appropriate callback
                        if change.removedTermId > 0 and self.remove_callback:
                            self.mem.pop(term.id, None)
                            self.remove_callback(change.addedOrRemovedTerm, change.source, change.stamp)
                        elif change.removedTermId == 0 and self.add_callback:
                            self.mem[term.id] = term
                            self.add_callback(term, change.source, change.stamp)
                    self.last_seq = resp.range_max
                else:
                    print "call to /memory/get failed"
            except rospy.ServiceException, e:
                print "call to /memory/get failed with exception: %s" % e

def print_memory():
    print "current memory: " + ", ".join(term_to_string(term) for term in ml.mem.values())

def callback_add(term, source, stamp):
    print "added term %s (id=%d) by '%s'" % (term_to_string(term), term.id, source)
    print_memory()

def callback_remove(term, source, stamp):
    print "removed term %s (id=%d) by '%s'" % (term_to_string(term), term.id, source)
    print_memory()

if __name__ == "__main__":
    global ml
    rospy.init_node('memory_listener')
    ml = MemoryListener(callback_add, callback_remove)
    print_memory()
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        r.sleep()

