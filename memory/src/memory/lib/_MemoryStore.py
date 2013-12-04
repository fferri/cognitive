#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import sys
import thread
import rospy

from ._common import *

from copy import copy

class MemoryStore:
    def __init__(self):
        self.seq = 1
        self.mem = {}
        self.log = []

    def new_term_id(self):
        ret = self.seq
        self.seq += 1
        return ret

    def add(self, term, src=''):
        if not term_is_ground(term):
            raise Exception('Cannot add unground term %s' % term_to_string(term))
        meta = TermMetadata()
        meta.term_id = self.new_term_id()
        meta.stamp = rospy.Time.now()
        meta.src = src
        meta.term = term
        self.mem[meta.term_id] = meta
        self.log.append(Change(op_type=Change.OP_ADD, src=src, meta=meta))
        return meta

    def remove_id(self, term_id, src=''):
        if term_id not in self.mem:
            raise Exception('Term id %d does not exist' % term_id)
        meta = copy(self.mem[term_id])
        meta.stamp = rospy.Time.now()
        self.log.append(Change(op_type=Change.OP_DEL, src=src, meta=meta))
        self.mem.pop(term_id, None)
        return meta

    def remove(self, term, src=''):
        ids_to_del = []
        for meta in self.mem.values():
            if term_equals(meta.term, term):
                ids_to_del.append(meta.term_id)
        ret = []
        for term_id in ids_to_del:
            ret.append(self.remove_id(term_id, src))
        return ret

    def change(self, term_id, new_term, src=''):
        if term_id not in self.mem:
            raise Exception('Term id %d does not exist' % term_id)
        meta = copy(self.mem[term_id])
        meta.stamp = rospy.Time.now()
        meta.src = src
        meta.term = new_term
        self.mem[term_id] = meta
        self.log.append(Change(op_type=Change.OP_CHANGE, src=src, meta=meta))
        return meta

    def log_size(self):
        return len(self.log)

    def log_get(self, range_min, range_max):
        if range_min < 1 or range_min > range_max or range_max > len(self.log):
            raise Exception('Indices out of range')
        return self.log[(range_min-1):range_max]

    def get_all(self):
        return self.mem.values()

