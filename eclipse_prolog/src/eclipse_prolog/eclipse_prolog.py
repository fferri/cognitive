#!/usr/bin/env python

import roslib; roslib.load_manifest('eclipse_prolog')
import rospy
import genpy
import roslib.message
import roslib.packages
import sys
import subprocess
import threading
import time
import rospy
import pyclp
import actionlib
import tf

class EclipseProlog:
    def __init__(self):
        pyclp.init()
        self.ecl_stream_stdin = pyclp.Stream('input')
        self.ecl_stream_stdout = pyclp.Stream('output')
        self.ecl_stream_stderr = pyclp.Stream('error')
        self.publishers = {}
        self.subscribers = {}
        self.topic_cache = {}
        self.action_clients = {}
        self.subprocesses = {}

        # compile helper predicates:
        self.compile(roslib.packages.get_pkg_dir('eclipse_prolog') + '/src/init.pl')

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
                data = self.ecl_stream_stderr.readall()
                if data:
                    sys.stderr.write(data)
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
    ## @param x The argument passed as first argument of yield/2
    ##
    ## @return The second argument to yield/2
    def yield_callback(self, x):
        if type(x) == pyclp.Atom:
            return self.yield_callback_atom(x)
        if type(x) == pyclp.Compound:
            return self.yield_callback_compound(x)
        if type(x) == pyclp.PList:
            return self.yield_callback_list(x)
        rospy.logwarn('yield callback: unhandled type: %s' % type(x))
        return pyclp.Var()

    ## @brief This is the yield callback specialized for atoms
    ##
    ## @param atom The atom argument passed as first argument of yield/2
    ##
    ## @return The second argument to yield/2
    def yield_callback_atom(self, atom):
        signature = 'yield_callback_%s_0' % atom
        if signature in dir(self):
            method = getattr(self, signature)
            return method()
        rospy.logwarn('yield callback: unhandled atom: %s' % atom)
        return pyclp.Var()

    ## @brief This is the yield callback specialized for compound terms
    ##
    ## @param compound The compound argument passed as first argument of yield/2
    ##
    ## @return The second argument to yield/2
    def yield_callback_compound(self, compound):
        signature = 'yield_callback_%s_%d' % (compound.functor(), compound.arity())
        if signature in dir(self):
            method = getattr(self, signature)
            args = list(compound.arguments())
            return method(*args)
        rospy.logwarn('yield callback: unhandled term: %s' % compound)
        return pyclp.Var()

    ## @brief This is the yield callback specialized for lists
    ##
    ## @param plist The list argument passed as first argument of yield/2
    ##
    ## @return The second argument to yield/2
    def yield_callback_list(self, plist):
        rospy.logwarn('yield callback: unhandled list: %s' % plist)
        return pyclp.Var()

    ## @brief Yield callback for advertise(TopicName, TopicType)
    def yield_callback_advertise_2(self, topic_name, topic_type):
        topic_name = str(topic_name)
        topic_type = str(topic_type)
        if not topic_name in self.publishers:
            topic_class = roslib.message.get_message_class(topic_type)
            self.publishers[topic_name] = rospy.Publisher(topic_name, topic_class)
        return pyclp.Var()

    ## @brief Yield callback for publish(TopicName, TopicType, Data)
    ## Data can be converted to a msg with term2msg()
    ## @see term2msg()
    def yield_callback_publish_3(self, topic_name, topic_type, topic_data):
        topic_name = str(topic_name)
        topic_type = str(topic_type)
        if not topic_name in self.publishers:
            topic_class = roslib.message.get_message_class(topic_type)
            self.publishers[topic_name] = rospy.Publisher(topic_name, topic_class)
        self.publishers[topic_name].publish(term2msg(topic_data))
        return pyclp.Var()

    ## @brief Yield callback for unpublish(TopicName)
    def yield_callback_unpublish_1(self, topic_name):
        topic_name = str(topic_name)
        if topic_name in self.publishers:
            self.publishers[topic_name].unregister()
            del self.publishers[topic_name]
        return pyclp.Var()

    ## @brief Yield callback for subscribe(TopicName, TopicType)
    def yield_callback_subscribe_2(self, topic_name, topic_type):
        topic_name = str(topic_name)
        topic_type = str(topic_type)
        if not topic_name in self.subscribers:
            topic_class = roslib.message.get_message_class(topic_type)
            self.subscribers[topic_name] = rospy.Subscriber(topic_name, topic_class, self.topic_callback, topic_name, 1)
        return pyclp.Var()

    ## @brief Yield callback for unsubscribe(TopicName)
    def yield_callback_unsubscribe_1(self, topic_name):
        topic_name = str(topic_name)
        if topic_name in self.subscribers:
            self.subscribers[topic_name].unregister()
            del self.subscribers[topic_name]
        return pyclp.Var()

    ## @brief Yield callback for get_topic(TopicName)
    def yield_callback_get_topic_1(self, topic_name):
        topic_name = str(topic_name)
        if topic_name in self.topic_cache:
            return msg2term(self.topic_cache[topic_name])
        return pyclp.Var()

    ## @brief Default callback for every ROS topic
    def topic_callback(self, data, topic_name):
        self.topic_cache[topic_name] = data

    ## @brief Yield callback for call_service(ServiceName, ServiceType, ServiceData, CallID)
    def yield_callback_call_service_4(self, service_name, service_type, service_data, call_id):
        service_name = str(service_name)
        service_type = str(service_type)
        call_id = str(call_id)
        service_class = roslib.message.get_service_class(service_type)
        service_proxy = rospy.ServiceProxy(service_name, service_class)
        rospy.logdebug('waiting for service %s (type: %s) to come up...' % (service_name, service_type))
        rospy.wait_for_service(service_name)
        rospy.logdebug('service %s (type: %s) is up' % (service_name, service_type))
        rospy.logdebug('calling service %s (type: %s) with id %s and with data: %s' % (service_name, service_type, call_id, service_data))
        resp = service_proxy(term2srv(service_data))
        resp = srv2term(resp)
        rospy.logdebug('service %s (type: %s) result for id %s: %s' % (service_name, service_type, call_id, resp))
        return resp

    ## @brief Callback for service result by call_service_async/4
    def call_service_async_callback(self, result, service_name, call_id):
        pass

    ## @brief Yield callback for call_service_async(ServiceName, ServiceType, ServiceData, CallID)
    def yield_callback_call_service_async_4(self, service_name, service_type, service_data, call_id):
        service_name = str(service_name)
        service_type = str(service_type)
        call_id = str(call_id)
        service_class = roslib.message.get_service_class(service_type)
        service_proxy = rospy.ServiceProxy(service_name, service_class)
        rospy.logdebug('waiting for service %s (type: %s) to come up...' % (service_name, service_type))
        rospy.wait_for_service(service_name)
        rospy.logdebug('service %s (type: %s) is up' % (service_name, service_type))

        def call_service_async_thread():
            rospy.logdebug('calling service %s (type: %s) asynchronously with id %s and with data: %s' % (service_name, service_type, call_id, service_data))
            resp = service_proxy(term2srv(service_data))
            rospy.logdebug('service %s (type: %s) result for id %s: %s' % (service_name, service_type, call_id, srv2term(resp)))
            self.call_service_async_callback(resp, service_name, call_id)
        threading.Thread(target=call_service_async_thread, args=()).start()
        return pyclp.Var()

    ## @brief Yield callback for action_send(ActionServerName, ActionServerType, GoalID, Data)
    def yield_callback_action_send_4(self, as_name, as_type, goal_id, data):
        as_name = str(as_name)
        as_type = str(as_type)
        goal_id = str(goal_id)
        rospy.logdebug('action_send(%s, %s, %s, %s)' % (as_name, as_type, goal_id, data))

        rospy.logdebug('resolving actionlib message class \'%s\'' % as_type)
        as_class = roslib.message.get_message_class(as_type + 'Action')
        rospy.logdebug('resolved to %s' % as_class)

        rospy.logdebug('creating a SimpleActionClient(%s, %s) for goal id \'%s\'' % (as_name, as_class, goal_id))
        self.action_clients[goal_id] = actionlib.SimpleActionClient(as_name, as_class)

        rospy.logdebug('waiting for actionlib server \'%s\'...' % as_name)
        self.action_clients[goal_id].wait_for_server()

        rospy.logdebug('sending goal id \'%s\': %s' % (goal_id, data))
        self.action_clients[goal_id].send_goal(term2msg(data),
            lambda state, result: self.action_done_callback(goal_id, state, result),
            lambda: self.action_active_callback(goal_id),
            lambda feedback: self.action_feedback_callback(goal_id, feedback))

        return pyclp.Atom(goal_id)

    ## @brief Yield callback for action_abort(GoalID)
    def yield_callback_action_abort_1(self, goal_id):
        goal_id = str(goal_id)
        if goal_id in self.action_clients:
            rospy.logdebug('aborting goal id \'%s\'...' % goal_id)
            self.action_clients[goal_id].cancel_goal()
            rospy.logdebug('aborted goal id \'%s\'' % goal_id)
            #TODO: notify someone about the action abort
        else:
            rospy.logwarn('cannot abort unexistsnt goal id \'%s\'' % goal_id)
        return pyclp.Var()

    ## @brief Yield callback for action_wait(GoalID)
    def yield_callback_action_wait_1(self, goal_id):
        goal_id = str(goal_id)
        if goal_id in self.action_clients:
            rospy.logdebug('waiting goal id \'%s\'...' % goal_id)
            client = self.action_clients[goal_id]
            #client.wait_for_result(rospy.Duration.from_sec(5.0))
            client.wait_for_result()
            result = client.get_result()
            status = actionlib.GoalStatus.to_string(client.get_state()).lower()
            rospy.logdebug('result of goal id \'%s\' is: %s' % (goal_id, result))
            rospy.logdebug('status for goal id \'%s\' is: %s' % (goal_id, status))
            #TODO: notify someone about the action end
            return msg2term(result)
        else:
            rospy.logwarn('cannot wait unexistent goal id \'%s\'' % goal_id)
            return pyclp.Var()

    ## @brief Yield callback for action_status(GoalID)
    def yield_callback_action_status_1(self, goal_id):
        goal_id = str(goal_id)
        if goal_id in self.action_clients:
            rospy.logdebug('retrieving status for goal id \'%s\'...' % goal_id)
            client = self.action_clients[goal_id]
            status = actionlib.GoalStatus.to_string(client.get_state()).lower()
            rospy.logdebug('status for goal id \'%s\' is: %s' % (goal_id, status))
            return pyclp.Atom(status)
        else:
            rospy.logwarn('cannot get status of unexistent goal id \'%s\'' % goal_id)
            return pyclp.Var()

    ## @brief Yield callback for action_status_simple(GoalID)
    def yield_callback_action_status_simple_1(self, goal_id):
        goal_id = str(goal_id)
        if goal_id in self.action_clients:
            rospy.logdebug('retrieving simple status for goal id \'%s\'...' % goal_id)
            client = self.action_clients[goal_id]
            status = actionlib.SimpleGoalState.to_string(client.simple_state)
            rospy.logdebug('simple status for goal id \'%s\' is: %s' % (goal_id, status))
            return pyclp.Atom(status)
        else:
            rospy.logwarn('cannot get simple status of unexistent goal id \'%s\'' % goal_id)
            return pyclp.Var()

    ## @brief This callback gets called on transitions to done
    def action_done_callback(self, goal_id, state, result):
        pass

    ## @brief This callback gets called on transitions to active
    def action_active_callback(self, goal_id):
        pass

    ## @brief This callback gets called whenever feedback for the goal is received
    def action_feedback_callback(self, goal_id, feedback):
        pass

    ## @brief Yield callback for quaternion_from_euler(RPY)
    def yield_callback_quaternion_from_euler_1(self, rpy):
        q = tf.transformations.quaternion_from_euler(*list(float(x) for x in rpy))
        return pyclp.PList(list(pyclp.Term(x) for x in q))

    ## @brief Yield callback for subprocess_open(ProcessID, Argv)
    def yield_callback_subprocess_open_2(self, process_id, args):
        process_id = str(process_id)
        if process_id in self.subprocesses and self.subprocesses[process_id].poll():
            rospy.logerror('subprocess \'%s\' already running' % process_id)
            return pyclp.Var()
        args = list(str(arg) for arg in args)
        rospy.logdebug('starting process id \'%s\' with args %s...' % (process_id, args))
        self.subprocesses[process_id] = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        threading.Thread(target=self.subprocess_reader_thread, args=(process_id,'stdout')).start()
        threading.Thread(target=self.subprocess_reader_thread, args=(process_id,'stderr')).start()
        threading.Thread(target=self.subprocess_wait_thread, args=(process_id,)).start()
        rospy.logdebug('started process id \'%s\'' % process_id)
        return pyclp.Var()

    ## @brief This thread reads subprocess' stdout line by line, and calls appropriate callbacks
    ## @see subprocess_output_callback()
    def subprocess_reader_thread(self, process_id, channel_name):
        rospy.logdebug('reader_thread[%s:%s]: started' % (process_id, channel_name))
        for line in iter(getattr(self.subprocesses[process_id], channel_name).readline, ''):
            line = line.rstrip('\n')
            rospy.logdebug('reader_thread[%s:%s]: %s' % (process_id, channel_name, line))
            self.subprocess_output_callback(process_id, channel_name, line)
        rospy.logdebug('reader_thread[%s:%s]: finished' % (process_id, channel_name))

    ## @brief This thread wait subprocess' exit code, and calls appropriate callback
    ## @see subprocess_end_callback()
    def subprocess_wait_thread(self, process_id):
        rospy.logdebug('wait_thread[%s]: started' % (process_id,))
        exit_code = self.subprocesses[process_id].wait()
        self.subprocess_end_callback(process_id, exit_code)
        #del self.subprocesses[process_id]
        rospy.logdebug('wait_thread[%s]: finished. exit code: %d' % (process_id, exit_code))

    ## @brief Yield callback for subprocess_wait(ProcessID)
    def yield_callback_subprocess_wait_1(self, process_id):
        process_id = str(process_id)
        if process_id not in self.subprocesses:
            rospy.logwarn('cannot wait on unexistent process id \'%s\'' % process_id)
            return pyclp.Var()
        rospy.logdebug('waiting on process id \'%s\'' % process_id)
        exit_code = self.subprocesses[process_id].wait()
        del self.subprocesses[process_id]
        rospy.logdebug('got exit code %d from process id \'%s\'' % (exit_code, process_id))
        return pyclp.Term(exit_code)

    ## @brief Yield callback for subprocess_kill(ProcessID)
    def yield_callback_subprocess_kill_1(self, process_id):
        process_id = str(process_id)
        if process_id in self.subprocesses:
            rospy.logdebug('trying to kill process id \'%s\'' % process_id)
            self.subprocesses[process_id].kill()
            #self.subprocesses[process_id].terminate()
            rospy.logdebug('killed process id \'%s\'' % process_id)
            return pyclp.Var()
        else:
            rospy.logwarn('cannot kill unexistent process id \'%s\'' % process_id)
            return pyclp.Var()

    ## @brief This callback gets called whenever output from subprocesses is read
    def subprocess_output_callback(self, process_id, stream, line):
        pass

    ## @brief This callback gets called whenever subprocess terminates
    def subprocess_end_callback(self, process_id, exit_code):
        pass


# ROS topic <-> Prolog utilities:
def msg2term(msg):
    if msg is None:
        return pyclp.Atom('none')
    if type(msg) in [float, int]:
        return pyclp.Term(msg)
    if type(msg) in [str]:
        return pyclp.Atom(msg)
    if type(msg) in [list, tuple]:
        return pyclp.PList(list(msg2term(x) for x in msg))
    if type(msg) == genpy.rostime.Time:
        return pyclp.Compound('time', msg.to_sec())
    return pyclp.Compound(msg._type, pyclp.PList(list(pyclp.Compound(slot, msg2term(getattr(msg, slot))) for slot in msg.__slots__)))

def term2msg(term):
    if type(term) == pyclp.Atom and str(term) == 'none':
        return None
    if type(term) in [float, int, str]:
        return term
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
    if type(term) == tuple and term == ():
        return []
    if term.functor() == 'time':
        time_arg = term.arguments().next()
        return rospy.Time.now() if str(time_arg) == 'now' else genpy.rostime.Time(float(time_arg))
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
        args.append(pyclp.Compound(slot, msg2term(getattr(srv_resp, slot))))
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

