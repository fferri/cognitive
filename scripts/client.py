#!/usr/bin/env python

import roslib

roslib.load_manifest('memory')

import sys
import thread

import rospy

from common import *

from std_msgs.msg import *
from memory.msg import *
from memory.srv import *

class MemoryClient:
    def __init__(self, src, ns='/memory'):
        self.src = src
        self.ns = ns
        self.srv_add = None
        self.srv_remove = None
        self.srv_change = None
        self.srv_poll = None
        self.srv_get = None
        self.srv_dump = None

    def add(self, term):
        if self.srv_add == None:
            self.srv_add = rospy.ServiceProxy(self.ns + '/add', ChangeTerm)
        rospy.wait_for_service(self.ns + '/add')
        req = ChangeTermRequest(src=self.src, meta=TermMetadata(term=term))
        resp = self.srv_add(req)
        if len(resp.metas) > 1:
            raise Exception('Add response is a WTF')
        return resp.metas[0]

    def remove_id(self, term_id):
        if self.srv_remove == None:
            self.srv_remove = rospy.ServiceProxy(self.ns + '/remove', ChangeTerm)
        rospy.wait_for_service(self.ns + '/remove')
        req = ChangeTermRequest(src=self.src, meta=TermMetadata(term_id=term_id))
        resp = self.srv_remove(req)
        if len(resp.metas) > 1:
            raise Exception('Add response is a WTF')
        return resp.metas[0]

    def remove(self, term):
        if self.srv_remove == None:
            self.srv_remove = rospy.ServiceProxy(self.ns + '/remove', ChangeTerm)
        rospy.wait_for_service(self.ns + '/remove')
        req = ChangeTermRequest(src=self.src, meta=TermMetadata(term=term))
        resp = self.srv_remove(req)
        return resp.metas

    def change(self, term_id, term):
        if self.srv_change == None:
            self.srv_change = rospy.ServiceProxy(self.ns + '/change', ChangeTerm)
        rospy.wait_for_service(self.ns + '/change')
        req = ChangeTermRequest(src=self.src, meta=TermMetadata(term_id=term_id, term=term))
        resp = self.srv_change(req)
        return resp.metas[0]

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

if __name__ == "__main__":
    print "?"

