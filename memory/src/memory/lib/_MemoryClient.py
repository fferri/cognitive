#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import sys
import thread
import rospy

from ._common import *

class MemoryClient:
    def __init__(self, src=rospy.get_name(), ns='/memory'):
        self.src = src
        self.ns = ns
        self.srv_read = None
        self.srv_write = None
        self.srv_remove = None
        self.srv_poll = None
        self.srv_get = None
        self.srv_dump = None

    def read(self, term_id):
        if self.srv_read == None:
            self.srv_read = rospy.ServiceProxy(self.ns + '/read', ReadTerm)
        rospy.wait_for_service(self.ns + '/read')
        req = ReadTermRequest(src=self.src, term_id=term_id)
        resp = self.srv_read(req)
        return resp.meta

    def write(self, term_id, term):
        if self.srv_write == None:
            self.srv_write = rospy.ServiceProxy(self.ns + '/write', WriteTerm)
        rospy.wait_for_service(self.ns + '/write')
        req = WriteTermRequest(src=self.src, meta=TermMetadata(term_id=term_id, term=term))
        resp = self.srv_write(req)

    def remove(self, term_id):
        if self.srv_remove == None:
            self.srv_remove = rospy.ServiceProxy(self.ns + '/remove', RemoveTerm)
        rospy.wait_for_service(self.ns + '/remove')
        req = RemoveTermRequest(src=self.src, term_id=term_id)
        resp = self.srv_remove(req)

    def log_size(self):
        if self.srv_poll == None:
            self.srv_poll = rospy.ServiceProxy(self.ns + '/poll', Dump)
        rospy.wait_for_service(self.ns + '/poll')
        req = DumpRequest()
        return self.srv_poll(req).logsize

    def log_get(self, range_min, range_max):
        if self.srv_get == None:
            self.srv_get = rospy.ServiceProxy(self.ns + '/get', Get)
        rospy.wait_for_service(self.ns + '/get')
        req = GetRequest(range_min=range_min, range_max=range_max)
        resp = self.srv_get(req)
        return resp.changes
        
    def get_all(self):
        if self.srv_dump == None:
            self.srv_dump = rospy.ServiceProxy(self.ns + '/dump', Dump)
        rospy.wait_for_service(self.ns + '/dump')
        req = DumpRequest()
        return self.srv_dump(req).metas

