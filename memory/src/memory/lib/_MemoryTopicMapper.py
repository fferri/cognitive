#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')

import rospy, threading, time, sys

from ._common import *
from ._MemoryClient import *
from ._MemoryStore import *

class MemoryTopicMapper:
    def __init__(self, topic_name, topic_class, term_id, expr, timeout=0, src=rospy.get_name(), ns='/memory'):
        self.client = MemoryClient(src=src, ns=ns)
        self.ns = ns
        self.last_pub_time = 0
        self.term_id = term_id
        self.last_term = None
        self.timeout = timeout * 0.001
        self.expr = expr
        self.client.remove(self.term_id)
        rospy.Subscriber(topic_name, topic_class, self.callback, queue_size=1)

    def callback(self, m):
        self.last_pub_time = time.time()
        t = eval(self.expr)
        self.client.write(self.term_id, t)

    def on_timeout(self):
        try:
            self.client.remove(self.term_id)
        except rospy.service.ServiceException:
            rospy.logwarn('Failed to delete old term with id %s' % self.term_id)

    def spin(self):
        if self.timeout > 0:
            age = time.time() - self.last_pub_time
            if age > self.timeout:
                self.on_timeout()

    def on_shutdown(self):
        self.on_timeout()

