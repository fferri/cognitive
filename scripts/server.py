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

seq = 1
mem = {}
log = []

def new_term_id():
    global seq
    ret = seq
    seq += 1
    return ret

def callback_add(req):
    global mem, log, seq
    term = req.term
    term.creator = req.source
    term.id = new_term_id()
    mem[term.id] = term
    log.append(Change(source=req.source, stamp=rospy.Time.now(), addedOrRemovedTerm=term))
    print "added term %s with id %d" % (term_to_string(term), term.id)
    return AddTermResponse(succeeded=True, id=term.id)

def callback_remove(req):
    global mem, log, seq
    if req.id in mem:
        term = mem[req.id]
        mem.pop(req.id, None)
        log.append(Change(source=req.source, stamp=rospy.Time.now(), removedTermId=req.id, addedOrRemovedTerm=term))
        print "removed term %s with id %d" % (term_to_string(term), term.id)
        return RemoveTermResponse(succeeded=True)
    else:
        return RemoveTermResponse(succeeded=False)

def callback_poll(req):
    return PollResponse(logsize=len(log))

def callback_get(req):
    rMin = req.range_min
    rMax = req.range_max
    if req.range_min < 1 or req.range_min > req.range_max or req.range_max > len(log):
        return GetResponse(succeeded=False, range_min=rMin, range_max=rMax)
    else:
        return GetResponse(succeeded=True, range_min=rMin, range_max=rMax, changes=log[(rMin-1):rMax])

def callback_dump(req):
    return DumpResponse(logsize=len(log), terms=mem.values())

def memory_server():
    rospy.init_node('memory_server')
    pubSeq    = rospy.Publisher('memory/seq', UInt32)
    srvAdd    = rospy.Service('memory/add', AddTerm, callback_add)
    srvRemove = rospy.Service('memory/remove', RemoveTerm, callback_remove)
    srvPoll   = rospy.Service('memory/poll', Poll, callback_poll)
    srvGet    = rospy.Service('memory/get', Get, callback_get)
    srvDump   = rospy.Service('memory/dump', Dump, callback_dump)

    # add something
    callback_add(AddTermRequest(source='self', term=TermX('foo', AtomInt(1))))

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pubSeq.publish(UInt32(data=len(log)))
        r.sleep()

if __name__ == "__main__":
    memory_server()

