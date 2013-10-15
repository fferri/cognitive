#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import sys
import thread
import rospy

from memory.lib import *

store = MemoryStore()

def callback_add(req):
    meta = store.add(req.meta.term, req.src)
    return ChangeTermResponse(metas=[meta])

def callback_remove(req):
    if req.meta.term_id > 0:
        metas = [store.remove_id(req.meta.term_id, req.src)]
    else:
        metas = store.remove(req.meta.term, req.src)
    return ChangeTermResponse(metas=metas)

def callback_change(req):
    meta = store.change(req.meta.term_id, req.meta.term, req.src)
    return ChangeTermResponse(metas=[meta])
        
def callback_poll(req):
    return DumpResponse(logsize=store.log_size())

def callback_get(req):
    log = store.log_get(req.range_min, req.range_max)
    return GetResponse(changes=log)

def callback_dump(req):
    return DumpResponse(logsize=store.log_size(), metas=store.get_all())

def memory_server():
    rospy.init_node('memory_server')
    ns = rospy.get_param('~memory_namespace', '/memory')
    pubSeq    = rospy.Publisher(ns + '/seq', UInt32)
    srvAdd    = rospy.Service(ns + '/add', ChangeTerm, callback_add)
    srvRemove = rospy.Service(ns + '/remove', ChangeTerm, callback_remove)
    srvChange = rospy.Service(ns + '/change', ChangeTerm, callback_change)
    srvPoll   = rospy.Service(ns + '/poll', Dump, callback_poll)
    srvGet    = rospy.Service(ns + '/get', Get, callback_get)
    srvDump   = rospy.Service(ns + '/dump', Dump, callback_dump)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pubSeq.publish(UInt32(data=store.log_size()))
        r.sleep()

if __name__ == "__main__":
    memory_server()

