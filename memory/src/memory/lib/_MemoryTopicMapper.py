#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')

import rospy, threading, time, sys

from ._common import *
from ._MemoryClient import *
from ._MemoryStore import *

class MemoryTopicMapper:
    def __init__(self, topic_name, topic_class, term_id, term_name, term_args, timeout=0, src=rospy.get_name(), ns='/memory'):
        self.client = MemoryClient(src=src, ns=ns)
        self.ns = ns
        self.last_pub_time = 0
        self.term_id = term_id
        self.last_term = None
        self.timeout = timeout * 0.001
        self.term_name = term_name
        self.term_args = term_args
        self.client.remove(self.term_id)
        rospy.Subscriber(topic_name, topic_class, self.callback, queue_size=1)

    def callback(self, m):
        self.last_pub_time = time.time()

        t = Term()
        t.functor = self.term_name
        for term_arg in self.term_args:
            t.args.append(atom_parse(eval(term_arg)))
        
        self.client.write(self.term_id, t)

    def on_timeout(self):
        try:
            self.client.remove(self.term_id)
        except rospy.service.ServiceException:
            rospy.logwarn('Failed to delete old term with id %d' % self.term_id)

    def spin(self):
        if self.timeout > 0:
            age = time.time() - self.last_pub_time
            if age > self.timeout:
                self.on_timeout()

    def on_shutdown(self):
        self.on_timeout()

