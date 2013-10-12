#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import sys
import thread
import rospy

from ._common import *
from ._MemoryClient import *
from ._MemoryStore import *

class MemoryListener:
    def __init__(self, ns='/memory'):
        self.client = MemoryClient('listener', ns)
        self.store = MemoryStore()
        self.ns = ns
        # get current memory snapshot
        log_size = self.client.log_size()
        metas = self.client.get_all()
        for meta in metas:
            self.store.mem[meta.term_id] = meta
        self.last_seq = log_size
        rospy.Subscriber(ns + '/seq', UInt32, self.callback_seq)

    def callback_seq(self, msg):
        if msg.data > self.last_seq:
            try:
                # get all edits
                changes = self.client.log_get(range_min=self.last_seq+1, range_max=msg.data)
                if changes == None:
                    print("call to %s/get failed" % self.ns)

                for change in changes:
                    term = change.meta.term
                    # notify the appropriate callback
                    if change.op_type == Change.OP_DEL:
                        self.store.mem.pop(change.meta.term_id, None)
                        self.callback_remove(change.meta, change.src)
                    elif change.op_type == Change.OP_ADD:
                        self.store.mem[change.meta.term_id] = change.meta
                        self.callback_add(change.meta, change.src)
                    elif change.op_type == Change.OP_CHANGE:
                        old_meta = self.store.mem[change.meta.term_id]
                        self.store.mem[change.meta.term_id] = change.meta
                        self.callback_change(old_meta, change.meta, change.src)
                    else:
                        rospy.logwarn('Unknown change op_type %d' % change.op_type)
                self.last_seq = msg.data
            except rospy.ServiceException, e:
                print("call to %s/get failed with exception: %s" % (self.ns, e))

    def callback_add(self, meta, src):
        pass

    def callback_remove(self, meta, src):
        pass

    def callback_change(self, meta, src):
        pass

