#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import sys
import thread
import rospy

from memory.lib import *

if __name__ == "__main__":
    global ml
    rospy.init_node('memory_listener')
    class MyListener(MemoryListener):
        def __init__(self, ns='/memory'):
            MemoryListener.__init__(self, ns)

        def callback_add(self, meta, src):
            print "%s added term %s (id=%d)" % (src, term_to_string(meta.term), meta.term_id)
            print_terms_table(ml.store.mem.values())

        def callback_remove(self, meta, src):
            print "%s removed term %s (id=%d)" % (src, term_to_string(meta.term), meta.term_id)
            print_terms_table(ml.store.mem.values())

        def callback_change(self, old_meta, meta, src):
            print "%s changed term %s to %s (id=%d)" % (src, term_to_string(old_meta.term), term_to_string(meta.term), meta.term_id)
            print_terms_table(ml.store.mem.values())

    ml = MyListener()
    print_terms_table(ml.store.mem.values())
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        r.sleep()

