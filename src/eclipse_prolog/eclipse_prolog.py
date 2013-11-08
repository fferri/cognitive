#!/usr/bin/env python

import roslib; roslib.load_manifest('eclipse_prolog')
import roslib.packages
import sys
import thread
import rospy
import pyclp

class EclipseProlog:
    def __init__(self):
        pyclp.init()
        self.ecl_stream_stdin = pyclp.Stream('input')
        self.ecl_stream_stdout = pyclp.Stream('output')
        self.ecl_stream_stderr = pyclp.Stream('error')

    def __del__(self):
        pyclp.cleanup()

    def resume(self):
        in_term=None
        while True:
            result,arg=pyclp.resume(in_term)
            in_term=None
            if result == pyclp.SUCCEED or result == pyclp.FAIL:
                return result
            if result == pyclp.FLUSHIO:
                data = self.ecl_stream_stdout.readall()
                if data:
                    sys.stdout.write(data)
            elif result == pyclp.WAITIO:
                print('eclipse: WAITIO: not implemented')
            elif result == pyclp.YIELD:
                in_term=self.yield_callback(arg)
            elif result == pyclp.THROW:
                print('eclipse: uncaught exception \'%s\'' % arg)
                return pyclp.FAIL
            else:
                raise Exception('eclipse: unknown result: %d' % result)

    def post_goal(self, term, exc=None):
        term.post_goal()
        result=self.resume()
        if exc and result != pyclp.SUCCEED:
            raise exc
        else:
            return result

    def compile(self, filename):
        self.post_goal(pyclp.Compound('compile', pyclp.Atom(filename)),
            Exception('Failed to compile %s' % filename))

    def yield_callback(self, x):
        if type(x) == pyclp.Atom:
            return self.yield_callback_atom(x)
        if type(x) == pyclp.Compound:
            return self.yield_callback_compound(x)
        if type(x) == pyclp.PList:
            return self.yield_callback_list(x)
        print('yield callback: unhandled type %s' % type(x))
        return pyclp.Atom('1')

    def yield_callback_atom(self, atom):
        return pyclp.Atom('1')

    def yield_callback_compound(self, compound):
        return pyclp.Atom('1')

    def yield_callback_list(self, plist):
        return pyclp.Atom('1')

