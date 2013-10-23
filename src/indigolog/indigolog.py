#!/usr/bin/env python

import roslib; roslib.load_manifest('indigolog')
import roslib.packages
import sys
import thread
import rospy
import pyclp

from memory.lib import *

class Indigolog:
    def __init__(self, indigologdir, mainfile, mainproc):
        pyclp.init()
        self.ecl_stream_stdin = pyclp.Stream('input')
        self.ecl_stream_stdout = pyclp.Stream('output')
        self.ecl_stream_stderr = pyclp.Stream('error')

        interp_pl = indigologdir + '/Interpreters/indigolog-vanilla_ecl.pl'
        self._compile(interp_pl)

        self._compile(mainfile)
        self._post_goal(pyclp.Atom(mainproc),
            Exception('Program terminated with error'))
        #print('Program terminated successfully')

        pyclp.cleanup()

    def _resume(self):
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
                print('eclipse: WAITIO. WTF?!')
            elif result == pyclp.YIELD:
                in_term=self.yield_callback(arg)
            elif result == pyclp.THROW:
                print('eclipse: uncaught exception \'%s\'' % arg)
                return pyclp.FAIL
            else:
                raise Exception('eclipse: unknown result: %d' % result)

    def _post_goal(self, term, exc=None):
        term.post_goal()
        result=self._resume()
        if exc and result != pyclp.SUCCEED:
            raise exc
        else:
            return result

    def _compile(self, filename):
        self._post_goal(pyclp.Compound('compile', pyclp.Atom(filename)),
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

    def yield_callback_list(self, atom):
        return pyclp.Atom('1')

if __name__ == "__main__":
    rospy.init_node('indigolog')
    indigologdir = rospy.get_param('~indigologdir', '/usr/local/indigolog')
    mainfile = rospy.get_param('~mainfile', roslib.packages.get_pkg_dir('indigolog') + '/src/main.pl')
    mainproc = rospy.get_param('~mainproc', 'main')
    indigolog = Indigolog(indigologdir, mainfile, mainproc)

