#!/usr/bin/env python

import roslib; roslib.load_manifest('eclipse_prolog')
import rospy
import genpy
import roslib.message
import roslib.packages
import sys
import thread
import rospy
import pyclp
import actionlib

class EclipseProlog:
    def __init__(self):
        pyclp.init()
        self.ecl_stream_stdin = pyclp.Stream('input')
        self.ecl_stream_stdout = pyclp.Stream('output')
        self.ecl_stream_stderr = pyclp.Stream('error')
        self.publishers = {}
        self.subscribers = {}
        self.topic_cache = {}
        self.services = {}
        self.action_clients = {}

    def __del__(self):
        pyclp.cleanup()

    ## @brief Resume Eclipse execution also handling any I/O operation, yield,
    ## and exceptions
    ##
    ## If execution interrupts due to a yield, the appropriate yield callback
    ## is called, and execution is resumed again.
    ##
    ## Likewise, if execution interrupts due to I/O, the I/O is handled, and
    ## execution resumes up to the end of the computation.
    ##
    ## @return The result of the computation (FAIL or SUCCEED)
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

    ## @brief Post a goal and resume execution
    ##
    ## @param term The goal
    ## 
    ## @param exc If specified, and the goal fails, throw this exception
    ##
    ## @return The exit value of the goal
    def post_goal(self, term, exc=None):
        term.post_goal()
        result=self.resume()
        if exc and result != pyclp.SUCCEED:
            raise exc
        else:
            return result

    ## @brief Compile the specified file
    ##
    ## @param filename The file to compile
    def compile(self, filename):
        self.post_goal(pyclp.Compound('compile', pyclp.Atom(filename)),
            Exception('Failed to compile %s' % filename))

    ## @brief This callback gets called when Eclipse execution interrupts
    ## due to a yield/1 or yield/2 call
    ##
    ## @param x The argument to return as second argument of yield/2
    def yield_callback(self, x):
        if type(x) == pyclp.Atom:
            return self.yield_callback_atom(x)
        if type(x) == pyclp.Compound:
            return self.yield_callback_compound(x)
        if type(x) == pyclp.PList:
            return self.yield_callback_list(x)
        rospy.logwarn('yield callback: unhandled type: %s' % type(x))
        return pyclp.Var()

    def yield_callback_atom(self, atom):
        signature = 'yield_callback_%s_0' % atom
        if signature in dir(self):
            method = getattr(self, signature)
            return method()
        rospy.logwarn('yield callback: unhandled atom: %s' % atom)
        return pyclp.Var()

    def yield_callback_compound(self, compound):
        signature = 'yield_callback_%s_%d' % (compound.functor(), compound.arity())
        if signature in dir(self):
            method = getattr(self, signature)
            args = list(compound.arguments())
            return method(*args)
        rospy.logwarn('yield callback: unhandled term: %s' % compound)
        return pyclp.Var()

    def yield_callback_list(self, plist):
        rospy.logwarn('yield callback: unhandled list: %s' % plist)
        return pyclp.Var()

    def yield_callback_publish_3(self, topic_name, topic_type, topic_data):
        topic_name = str(topic_name)
        topic_type = str(topic_type)
        if not topic_name in self.publishers:
            topic_class = roslib.message.get_message_class(topic_type)
            self.publishers[topic_name] = rospy.Publisher(topic_name, topic_class)
        self.publishers[topic_name].publish(term2msg(topic_data))
        return pyclp.Var()

    def yield_callback_unpublish_1(self, topic_name):
        topic_name = str(topic_name)
        if topic_name in self.publishers:
            self.publishers[topic_name].unregister()
            del self.publishers[topic_name]
        return pyclp.Var()

    def yield_callback_subscribe_2(self, topic_name, topic_type):
        topic_name = str(topic_name)
        topic_type = str(topic_type)
        if not topic_name in self.subscribers:
            topic_class = roslib.message.get_message_class(topic_type)
            self.subscribers[topic_name] = rospy.Subscriber(topic_name, topic_class, self.topic_callback, topic_name, 1)
        return pyclp.Var()

    def yield_callback_unsubscribe_1(self, topic_name):
        topic_name = str(topic_name)
        if topic_name in self.subscribers:
            self.subscribers[topic_name].unregister()
            del self.subscribers[topic_name]
        return pyclp.Var()

    def yield_callback_get_topic_1(self, topic_name):
        topic_name = str(topic_name)
        if topic_name in self.topic_cache:
            return msg2term(self.topic_cache[topic_name])
        return pyclp.Var()

    def topic_callback(self, data, topic_name):
        self.topic_cache[topic_name] = data

    def yield_callback_call_service_3(self, service_name, service_type, service_data):
        service_name = str(service_name)
        service_type = str(service_type)
        if not service_name in self.services:
            service_class = roslib.message.get_service_class(service_type)
            self.services[service_name] = rospy.ServiceProxy(service_name, service_class)
        rospy.wait_for_service(service_name)
        resp = self.services[service_name](term2srv(service_data))
        return srv2term(resp)

    def yield_callback_action_send_4(self, as_name, as_type, goal_id, data):
        as_name = str(as_name)
        as_type = str(as_type)
        goal_id = str(goal_id)
        if goal_id in self.action_clients:
            raise Exception('goal \'%s\' already pending')
        as_class = roslib.message.get_message_class(as_type + 'Action')
        self.action_clients[goal_id] = actionlib.SimpleActionClient(as_name, as_class)
        self.action_clients[goal_id].wait_for_server()
        self.action_clients[goal_id].send_goal(term2msg(data))
        return pyclp.Atom(goal_id)

    def yield_callback_action_abort_1(self, goal_id):
        self.action_clients[goal_id].cancel_goal()
        #TODO: notify someone about the action abort
        return pyclp.Var()

    def yield_callback_action_wait_1(self, goal_id):
        #self.action_clients[goal_id].wait_for_result(rospy.Duration.from_sec(5.0))
        self.action_clients[goal_id].wait_for_result()
        result = self.action_clients[goal_id].get_result()
        #TODO: notify someone about the action end
        return pyclp.Var()

    def yield_callback_action_status_0(self):
        self.get_finished_goals()

    def get_finished_goals(self):
        results = {}
        print('*** get_finished_goals ***')
        for (goal_id, client) in self.action_clients.items():
            print('%s: state=%s, simple_state=%d, result=%s' % (goal_id, client.get_state(), client.simple_state, client.get_result()))
            if client.simple_state == actionlib.SimpleGoalState.DONE:
                client.wait_for_result()
                results[goal_id] = client.get_result()
        print('**************************')

        # purge:
        for (goal_id, result) in results.items():
            del self.action_clients[goal_id]
            self.action_result_callback(goal_id, result)

    def action_result_callback(self, goal_id, result):
        pass

# ROS topic <-> Prolog utilities:
def msg2term(msg):
    if type(msg) in [float, int, str]:
        return msg
    if type(msg) in [list, tuple]:
        return pyclp.PList(list(msg2term(x) for x in msg))
    if type(msg) == genpy.rostime.Time:
        return pyclp.Compound('time', msg.to_sec())
    return pyclp.Compound(msg._type, pyclp.PList(list(pyclp.Compound(slot, msg2term(getattr(msg, slot))) for slot in msg.__slots__)))

def term2msg(term):
    if type(term) == pyclp.PList:
        return list(term2msg(x) for x in term)
    if type(term) == pyclp.Atom:
        try:
            return int(term)
        except:
            try:
                return float(term)
            except:
                return str(term)
    if term.functor() == 'time':
        return genpy.rostime.Time(float(term.arguments().next()))
    topic_class = roslib.message.get_message_class(term.functor())
    prolog_dict = term.arguments().next()
    kwargs = {}
    for kv in prolog_dict:
        kwargs[kv.functor()] = term2msg(kv.arguments().next())
    return topic_class(**kwargs)

# ROS service <-> Prolog utilities:
def srv2term(srv_resp):
    args = []
    for slot in srv_resp.__slots__:
        slot_obj = getattr(srv_resp, slot)
        if not type(slot_obj) in [float, int, str]:
            slot_obj = msg2term(slot_obj)
        args.append(pyclp.Compound(slot, slot_obj))
    return pyclp.Compound(srv_resp._type, pyclp.PList(args))

def term2srv(term):
    if type(term) in [float, int, str]:
        return term
    srv_resp_class = roslib.message.get_service_class(term.functor())._request_class
    prolog_dict = term.arguments().next()
    kwargs = {}
    for kv in prolog_dict:
        kwargs[kv.functor()] = term2msg(kv.arguments().next())
    return srv_resp_class(**kwargs)

