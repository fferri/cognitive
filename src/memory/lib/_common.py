#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import sys
import thread
import re
import rospy

from std_msgs.msg import *
from memory.msg import *
from memory.srv import *

def AtomNull():
    return Atom(intData=[], floatData=[], stringData=[])

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
    if t > 1:
        raise TypeError("atom_check: at most one of intData floatData stringData must have one element (total: %d)" % t)

def atom_is_null(a):
    return len(a.intData) == 0 and len(a.floatData) == 0 and len(a.stringData) == 0

def atom_is_int(a):
    return len(a.intData) > 0

def atom_is_float(a):
    return len(a.floatData) > 0

def atom_is_string(a):
    return len(a.stringData) > 0

def atom_get_int(a):
    return a.intData[0]

def atom_get_float(a):
    return a.floatData[0]

def atom_get_string(a):
    return a.stringData[0]

def atom_get(a):
    if atom_is_int(a): return atom_get_int(a)
    if atom_is_float(a): return atom_get_float(a)
    if atom_is_string(a): return atom_get_string(a)
    return None

def atom_equals(a1,a2):
    atom_check(a1)
    atom_check(a2)
    if atom_is_null(a1) or atom_is_null(a2):
        return True
    if len(a1.intData) != len(a2.intData) or len(a1.floatData) != len(a2.floatData) or len(a1.stringData) != len(a2.stringData):
        return False
    for (i,n) in enumerate(a1.intData):
        if a2.intData[i] != n: return False
    for (i,f) in enumerate(a1.floatData):
        if a2.floatData[i] != f: return False
    for (i,s) in enumerate(a1.stringData):
        if a2.stringData[i] != s: return False
    return True

def atom_parse(anything):
    if type(anything) == int:
        return AtomInt(anything)
    elif type(anything) == float:
        return AtomFloat(anything)
    elif type(anything) == str:
        if anything == '_':
            return AtomNull()
        try:
            return AtomInt(int(anything))
        except:
            try:
                return AtomFloat(float(anything))
            except:
                return AtomString(anything)
    else:
        raise Exception('cannot make an atom from a %s' % type(anything))

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

def term_parse(str_list):
    functor = str_list[0]
    args = []
    for arg in str_list[1:]:
        args.append(atom_parse(arg))
    return Term(functor=functor, args=args)

def term_to_string(t):
    if type(t) != Term:
        print "term_to_string: WTF? (%s, type=%s)" % (t, type(t))
    if len(t.args) > 0:
        return "%s(%s)" % (t.functor, ", ".join(atom_to_string(a) for a in t.args))
    else:
        return t.functor

def term_equals(t1,t2):
    if t1.functor != t2.functor: return False
    if len(t1.args) != len(t2.args): return False
    for i in range(0,len(t1.args)):
        if not atom_equals(t1.args[i],t2.args[i]):
            return False
    return True

def term_is_ground(t):
    if t.functor == '':
        return False
    for arg in t.args:
        if atom_is_null(arg):
            return False
    return True

def get_terminal_size():
    import os
    env = os.environ
    def ioctl_GWINSZ(fd):
        try:
            import fcntl, termios, struct, os
            cr = struct.unpack('hh', fcntl.ioctl(fd, termios.TIOCGWINSZ,
        '1234'))
        except:
            return
        return cr
    cr = ioctl_GWINSZ(0) or ioctl_GWINSZ(1) or ioctl_GWINSZ(2)
    if not cr:
        try:
            fd = os.open(os.ctermid(), os.O_RDONLY)
            cr = ioctl_GWINSZ(fd)
            os.close(fd)
        except:
            pass
    if not cr:
        cr = (env.get('LINES', 25), env.get('COLUMNS', 80))

        ### Use get(key[, default]) instead of a try/catch
        #try:
        #    cr = (env['LINES'], env['COLUMNS'])
        #except:
        #    cr = (25, 80)
    return int(cr[1]), int(cr[0])

def print_terms_table(entries):
    (width, height) = get_terminal_size()    
    #colw = [8, -50, -12]
    src_w = int(width/8)
    colw = [8, -(width - 10 - src_w - 8), -src_w]
    fmt = '| ' + ' | '.join("%%%ds" % w for w in colw) + ' |'
    hline = '+' + '+'.join('-' * (abs(w) + 2) for w in colw) + '+'
    print hline
    print fmt % ('id', 'term', 'source')
    print hline
    for entry in entries:
        if type(entry) == TermMetadata:
            term = entry.term
            term_id = str(entry.term_id)
            src = entry.src
        elif type(entry) == Term:
            term = entry
            term_id = '?'
            src = 'N/A'
        print fmt % (term_id, term_to_string(term), src)    
    print hline

