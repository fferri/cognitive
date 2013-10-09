#!/usr/bin/env python

import roslib

roslib.load_manifest('memory')

import sys
import thread

import rospy

from std_msgs.msg import *
from memory.msg import *
from memory.srv import *

def AtomInt(i):
    return Atom(intData=[i], floatData=[], stringData=[])

def AtomFloat(f):
    return Atom(intData=[], floatData=[f], stringData=[])

def AtomString(s):
    return Atom(intData=[], floatData=[], stringData=[s])

def atom_check(a):
    if type(a) != Atom:
        raise TypeError("atom_check: type of a is %s" % type(a))
    if type(a.intData) not in [list,tuple]:
        raise TypeError("atom_check: type of a.intData is %s (value: %s)" % (type(a.intData), a.intData))
    if type(a.floatData) not in [list,tuple]:
        raise TypeError("atom_check: type of a.floatData is %s (value: %s)" % (type(a.floatData), a.floatData))
    if type(a.stringData) not in [list,tuple]:
        raise TypeError("atom_check: type of a.stringData is %s (value: %s)" % (type(a.stringData), a.stringData))
    if len(a.intData) not in [0,1]:
        raise TypeError("atom_check: length of a.intData is %s (value: %s)" % (len(a.intData), a.intData))
    if len(a.floatData) not in [0,1]:
        raise TypeError("atom_check: length of a.floatData is %s (value: %s)" % (len(a.floatData), a.floatData))
    if len(a.stringData) not in [0,1]:
        raise TypeError("atom_check: length of a.stringData is %s (value: %s)" % (len(a.stringData), a.stringData))
    t = len(a.intData) + len(a.floatData) + len(a.stringData)
    if t != 1:
        raise TypeError("atom_check: only one of intData floatData stringData must have one element (total: %d)" % t)

def atom_equals(a1,a2):
    atom_check(a1)
    atom_check(a2)
    if len(a1.intData) != len(a2.intData) or len(a1.floatData) != len(a2.floatData) or len(a1.stringData) != len(a2.stringData):
        return False
    for (i,n) in enumerate(a1.intData):
        if a2.intData[i] != n: return False
    for (i,f) in enumerate(a1.floatData):
        if a2.floatData[i] != f: return False
    for (i,s) in enumerate(a1.stringData):
        if a2.stringData[i] != s: return False
    return True

def atom_to_string(a):
    atom_check(a)
    if len(a.intData) == 1:
        return str(a.intData[0])
    if len(a.floatData) == 1:
        return str(a.floatData[0])
    if len(a.stringData) == 1:
        return a.stringData[0]
    return None

def TermX(f, *a):
    return Term(functor=f, args=a)

def term_to_string(t):
    if type(t) != Term:
        print "term_to_string: WTF? (%s, type=%s)" % (t, type(t))
    if len(t.args) > 0:
        return "%s(%s)" % (t.functor, ", ".join(atom_to_string(a) for a in t.args))
    else:
        return t.functor

def term_equals(t1,t2):
    if t1.functor != t2.functor: return False
    if len(t1.args) != (t2.args): return False
    for (i,a) in t1.args:
        if not atom_equals(a,t2.args[i]):
            return False
    return True

