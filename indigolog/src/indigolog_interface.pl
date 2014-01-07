:- lib(multifile).
:- set_flag(debug_compile, off).
:- set_flag(variable_names, off).  % To speed up execution
:- multifile(prim_action/1).
:- multifile(causes_val/4).
:- multifile(poss/2).
:- multifile(proc/2).
:- multifile(prim_fluent/1).
:- multifile(final/2).
:- multifile(trans/4).
:- multifile(execute/2).
:- multifile(exog_occurs/1).
:- multifile(initially/2).
:- ['/opt/indigolog/Interpreters/indigolog-vanilla'].
:- multifile(indigolog_trace/1).

indigolog_trace(exog) :- indigolog_trace(all).
indigolog_trace(exec) :- indigolog_trace(all).
indigolog_trace(sens) :- indigolog_trace(all).

%for simulation:
%execute(A,R) :- ask_execute(A,R).
%exog_occurs(A) :- ask_exog_occurs(A).

%for deploy:
execute(A,R) :- functor(A,F,N), ros_action(F/N), print_exec(A), yield(A,R), print_sens(A,R).
execute(A,_) :- member(A,[start_interrupts, stop_interrupts]).
exog_occurs(A) :- yield(check_exog_occurs,A), print_exog(A).

print_exog(none) :- !.
print_exog(A) :- indigolog_trace(exog), !, write('*** exogenous: '), writeln(A).
print_exog(_).

print_exec(A) :- indigolog_trace(exec), !, write('*** execute: '), writeln(A).
print_exec(_).

print_sens(A,R) :- indigolog_trace(sens), senses(A,_), !, write('*** sensing result of '), write(A), write(': '), writeln(R).
print_sens(_,_).

ros_action(advertise/2).
ros_action(publish/3).
ros_action(unpublish/1).
ros_action(subscribe/2).
ros_action(unsubscribe/1).
ros_action(call_service/3).
ros_action(call_service_async/4).
ros_action(action_send/4).
ros_action(action_abort/1).
ros_action(action_wait/1).
ros_action(action_status/1).
ros_action(action_status_simple/1).
ros_action(say/1).
ros_action(sleep/1).
ros_action(memory_read/1).
ros_action(memory_write/2).
ros_action(memory_remove/1).
ros_action(subprocess_open/2).
ros_action(subprocess_wait/1).
ros_action(subprocess_kill/1).

ros_exog_action(topic/2).
ros_exog_action(action_end/3).
ros_exog_action(action_feedback/2).
ros_exog_action(memory_add/3).
ros_exog_action(memory_remove/2).
ros_exog_action(memory_change/3).
ros_exog_action(subprocess_output/3).
ros_exog_action(subprocess_end/2).
ros_exog_action(service_async_result/3).

prim_action(A) :- functor(A,F,N), ros_action(F/N).
poss(A, true) :- functor(A,F,N), ros_action(F/N).

senses(memory_read(X),X).
senses(call_service(SrvName,_,_),SrvName) :- prim_fluent(SrvName).
senses(action_wait(G),F) :- F=action_result(G), prim_fluent(F).
senses(action_status(G),F) :- F=action_status(G), prim_fluent(F).
senses(action_status_simple(G),F) :- F=action_status_simple(G), prim_fluent(F).
senses(subprocess_wait(P),F) :- F=subprocess_exit_code(P), prim_fluent(F).

exog_action(A) :- functor(A,F,N), ros_exog_action(F/N).
poss(A, true) :- functor(A,F,N), ros_exog_action(F/N).

causes_val(topic(N,V), N, V, true) :- prim_fluent(N).
causes_val(memory_add(K,V,_), K, V, true) :- prim_fluent(K).
causes_val(memory_change(K,V,_), K, V, true) :- prim_fluent(K).
causes_val(memory_remove(K,_), K, nil, true) :- prim_fluent(K).
causes_val(service_async_result(R,SrvName,_CallId), SrvName, R, true) :- prim_fluent(SrvName).
causes_val(subprocess_end(P,C), F, C, true) :- F=subprocess_exit_code(P), prim_fluent(F).

proc(repeat(1, P), P) :- !.
proc(repeat(NTimes, P), [P, repeat(NTimesMinusOne, P)]) :- NTimes > 1, succ(NTimesMinusOne, NTimes).
