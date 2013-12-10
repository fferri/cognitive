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

    def contains(self, term_id, src=''):
        return term_id in self.mem

    def keys():
        return self.mem.keys()

    def read(self, term_id, src=''):
        if not self.contains(term_id):
            raise Exception('Term id %s does not exist' % term_id)
        return self.mem[term_id]

    def write(self, term_id, term, src=''):
        if not term_is_ground(term):
            raise Exception('Cannot add unground term %s' % term_to_string(term))

        meta = TermMetadata()
        if not term_id:
            term_id = '_%d' % self.seq
            self.seq += 1
        meta.term_id = term_id
        meta.stamp = rospy.Time.now()
        meta.src = src
        meta.term = term

        if self.contains(term_id):
            self.log.append(Change(op_type=Change.OP_CHANGE, src=src, meta=meta))
        else:
            self.log.append(Change(op_type=Change.OP_ADD, src=src, meta=meta))

        self.mem[meta.term_id] = meta
        return meta

    def remove(self, term_id, src=''):
        if not self.contains(term_id):
            raise Exception('Term id %s does not exist' % term_id)
        meta = copy(self.mem[term_id])
        meta.stamp = rospy.Time.now()
        self.log.append(Change(op_type=Change.OP_DEL, src=src, meta=meta))
        self.mem.pop(term_id, None)
        return meta

    def log_size(self):
        return len(self.log)

    def log_get(self, range_min, range_max):
        if range_min < 1 or range_min > range_max or range_max > len(self.log):
            raise Exception('Indices out of range')
        return self.log[(range_min-1):range_max]

    def get_all(self):
        return self.mem.values()

