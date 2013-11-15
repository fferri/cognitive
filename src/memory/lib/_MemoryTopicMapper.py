#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')

import rospy, threading, time, sys

from ._common import *
from ._MemoryClient import *
from ._MemoryStore import *

class MemoryTopicMapper:
    def __init__(self, topic_name, topic_class, term_name, term_args, timeout=0, src=rospy.get_name(), ns='/memory'):
        self.client = MemoryClient(src=src, ns=ns)
        self.ns = ns
        self.last_pub_time = 0
        self.term_id = 0
        self.last_term = None
        self.timeout = timeout * 0.001
        self.term_name = term_name
        self.term_args = term_args
        self.remove_old_terms()
        rospy.Subscriber(topic_name, topic_class, self.callback, queue_size=1)

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

    def write_term_to_memory(self, t):
        if self.term_id > 0:
            self.client.change(self.term_id, t)
        else:
            meta = self.client.add(t)
            self.term_id = meta.term_id

    def callback(self, msg):
        self.last_pub_time = time.time()
        t = self.make_term(msg)
        self.write_term_to_memory(t)

    def on_timeout(self):
        #self.remove_old_terms()
        if self.term_id > 0:
            try:
                self.client.remove_id(self.term_id)
                self.term_id = 0
            except rospy.service.ServiceException:
                rospy.logwarn('Failed to delete old term with id %d' % self.term_id)

    def spin(self):
        if self.timeout > 0:
            age = time.time() - self.last_pub_time
            if age > self.timeout:
                self.on_timeout()

    def on_shutdown(self):
        self.on_timeout()

