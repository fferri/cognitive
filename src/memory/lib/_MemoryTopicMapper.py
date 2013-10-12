#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')

import rospy, threading, time, sys

from ._common import *
from ._MemoryClient import *
from ._MemoryStore import *

class MemoryTopicMapper:
    def __init__(self, topic_name, topic_type, term_name, term_args, ns='/memory'):
        self.client = MemoryClient('mapper', ns)
        self.ns = ns
        self.last_pub_time = 0
        self.term_name = term_name
        self.term_args = term_args
        rospy.Subscriber(topic_name, topic_type, self.callback)

    def remove_old_terms(self):
        t = Term()
        t.functor = self.term_name
        t.args = []
        for term_arg in self.term_args:
            t.args.append(AtomNull())
        self.client.remove(t)

    def make_term(self, msg):
        t = Term()
        t.functor = self.term_name
        self.fill_term_args(t, msg)
        return t

    def fill_term_args(self, t, m):
        for term_arg in self.term_args:
            t.args.append(atom_parse(eval(term_arg)))

    def callback(self, msg):
        self.last_pub_time = time.time()
        self.remove_old_terms()
        t = self.make_term(msg)
        self.client.add(t)

