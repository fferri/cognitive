#!/usr/bin/env python

import roslib; roslib.load_manifest('memory')
import sys
import thread
import re
import rospy
import pyclp

from std_msgs.msg import *
from memory.msg import *
from memory.srv import *

def TokenInt(i):
    return Token(tokenType=Token.TYPE_INT, intData=int(i))

def TokenFloat(f):
    return Token(tokenType=Token.TYPE_FLOAT, floatData=float(f))

def TokenString(s):
    return Token(tokenType=Token.TYPE_STRING, stringData=str(s))

def TokenList(length):
    return Token(tokenType=Token.TYPE_LIST, intData=int(length))

def TokenFunctor(functor, arity):
    return Token(tokenType=Token.TYPE_FUNCTOR, intData=int(arity), stringData=str(functor))

def TermInt(i):
    return Term(tokens=[TokenInt(i)])

def TermFloat(f):
    return Term(tokens=[TokenFloat(f)])

def TermString(s):
    return Term(tokens=[TokenString(s)])

def TermList(*args):
    toks = []
    for arg in args:
        toks += arg.tokens
    return Term(tokens=[TokenList(len(args))] + toks)

def TermCompound(functor, *args):
    toks = []
    for arg in args:
        toks += arg.tokens
    return Term(tokens=[TokenFunctor(functor, len(args))] + toks)

def pyclpterm2termmsg(t):
    return Term(tokens=pyclpterm2tokenmsglist(t))

def termmsg2pyclpterm(m):
    return tokenmsglist2pyclpterm(m.tokens)

def pyclpterm2tokenmsglist(t):
    if type(t) == pyclp.PList:
        ret = [Token(tokenType=Token.TYPE_LIST, intData=len(t))]
        for arg in t:
            ret += pyclpterm2tokenmsglist(arg)
        return ret
    elif type(t) == pyclp.Compound:
        ret = [Token(tokenType=Token.TYPE_FUNCTOR, stringData=t.functor(), intData=t.arity())]
        for arg in t.arguments():
            ret += pyclpterm2tokenmsglist(arg)
        return ret
    elif type(t) == pyclp.Atom:
        try:
            return [Token(tokenType=Token.TYPE_INT, intData=int(t))]
        except:
            try:
                return [Token(tokenType=Token.TYPE_FLOAT, floatData=float(t))]
            except:
                return [Token(tokenType=Token.TYPE_STRING, stringData=str(t))]
    elif type(t) == int:
        return [Token(tokenType=Token.TYPE_INT, intData=t)]
    elif type(t) == float:
        return [Token(tokenType=Token.TYPE_FLOAT, floatData=t)]
    elif type(t) == str:
        return [Token(tokenType=Token.TYPE_STRING, stringData=t)]

def tokenmsglist2pyclpterm(l):
    term, rest = tokenmsglist2pyclptermRec(l)
    if rest == []:
        return term
    else:
        raise Exception('Malformed token list')

def tokenmsglist2pyclptermRec(l):
    head = l[0]
    l = l[1:]
    if head.tokenType == Token.TYPE_LIST:
        tlist = []
        for i in range(head.intData):
            term, l = tokenmsglist2pyclptermRec(l)
            tlist.append(term)
        return (pyclp.PList(tlist), l)
    elif head.tokenType == Token.TYPE_FUNCTOR:
        args = []
        for i in range(head.intData):
            term, l = tokenmsglist2pyclptermRec(l)
            args.append(term)
        return (pyclp.Compound(head.stringData, *args), l)
    elif head.tokenType == Token.TYPE_INT:
        #return (pyclp.Term(head.intData), l)
        return (head.intData, l)
    elif head.tokenType == Token.TYPE_FLOAT:
        #return (pyclp.Term(head.floatData), l)
        return (head.floatData, l)
    elif head.tokenType == Token.TYPE_STRING:
        return (pyclp.Atom(head.stringData), l)

def tokenmsg2str(tok):
    if tok.tokenType == Token.TYPE_INT: return str(tok.intData)
    elif tok.tokenType == Token.TYPE_FLOAT: return str(tok.floatData)
    elif tok.tokenType == Token.TYPE_STRING: return tok.stringData
    elif tok.tokenType == Token.TYPE_FUNCTOR: return ':%s/%d' % (tok.stringData, tok.intData)
    elif tok.tokenType == Token.TYPE_LIST: return ':%d' % tok.intData

def str2tokenmsg(s):
    m = re.match(r'^:(\d+)$', s)
    if m: return Token(tokenType=Token.TYPE_LIST, intData=int(m.group(1)))
    m = re.match(r'^:(.*)/(\d+)$', s)
    if m: return Token(tokenType=Token.TYPE_FUNCTOR, intData=int(m.group(2)), stringData=m.group(1))
    try:
        return Token(tokenType=Token.TYPE_INT, intData=int(s))
    except:
        try:
            return Token(tokenType=Token.TYPE_FLOAT, floatData=float(s))
        except:
            return Token(tokenType=Token.TYPE_STRING, stringData=str(s))

def tokenmsglist2str(l):
    return ' '.join(list(tokenmsg2str(tok) for tok in l))

def tokenmsglist2strPrettyRec(l):
    head = l[0]
    l = l[1:]
    if head.tokenType == Token.TYPE_LIST:
        tlist = []
        for i in range(head.intData):
            term, l = tokenmsglist2strPrettyRec(l)
            tlist.append(term)
        return ('[%s]' % ', '.join(tlist), l)
    elif head.tokenType == Token.TYPE_FUNCTOR:
        args = []
        for i in range(head.intData):
            term, l = tokenmsglist2strPrettyRec(l)
            args.append(term)
        return ('%s(%s)' % (head.stringData, ', '.join(args)), l)
    elif head.tokenType == Token.TYPE_INT:
        return (str(head.intData), l)
    elif head.tokenType == Token.TYPE_FLOAT:
        return (str(head.floatData), l)
    elif head.tokenType == Token.TYPE_STRING:
        return ('\'%s\'' % head.stringData, l)

def termmsg2str(term):
    return tokenmsglist2strPrettyRec(term.tokens)[0]

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
    id_w = max([8] + list(len(m.term_id) for m in entries))
    src_w = max([8] + list(len(m.src) for m in entries))
    colw = [id_w, -(width - 10 - src_w - id_w), -src_w]
    fmt = '| ' + ' | '.join("%%%ds" % w for w in colw) + ' |'
    hline = '+' + '+'.join('-' * (abs(w) + 2) for w in colw) + '+'
    print hline
    print fmt % ('id', 'term', 'source')
    print hline
    for entry in entries:
        if type(entry) == TermMetadata:
            term = entry.term
            term_id = entry.term_id
            src = entry.src
        elif type(entry) == Term:
            term = entry
            term_id = '?'
            src = 'N/A'
        print fmt % (term_id, termmsg2str(term), src)
    print hline

