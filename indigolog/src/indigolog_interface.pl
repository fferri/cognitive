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
:- ['indigolog/Interpreters/indigolog-vanilla'].

%for simulation:
%execute(A,R) :- ask_execute(A,R).
%exog_occurs(A) :- ask_exog_occurs(A).

%for deploy:
execute(A,R) :- functor(A,F,N), ros_action(F/N), yield(A,R).
execute(A,_) :- member(A,[start_interrupts, stop_interrupts]).
exog_occurs(A) :- yield(check_exog_occurs,A).

% for accessing msg/srv terms:
dict_get(D,[],D).
dict_get(D,[K|Ks],X) :- D =.. [_Type,Dict], member(KVPair,Dict), KVPair =.. [K,V], dict_get(V,Ks,X).

ros_action(advertise/2).
ros_action(publish/3).
ros_action(unpublish/1).
ros_action(subscribe/2).
ros_action(unsubscribe/1).
ros_action(call_service/3).
ros_action(say/1).
ros_action(sleep/1).
ros_action(memory_read/1).
ros_action(memory_write/2).
ros_action(memory_remove/1).

ros_exog_action(topic/2).
ros_exog_action(memory_add/3).
ros_exog_action(memory_remove/2).
ros_exog_action(memory_change/3).

prim_action(A) :- functor(A,F,N), ros_action(F/N).
poss(A, true) :- functor(A,F,N), ros_action(F/N).

%exog_occurs(_) :- !,fail.

senses(memory_read(X),X).

senses(call_service(N,_,_),N) :- prim_fluent(N).

%senses(_,_) :- fail.

exog_action(A) :- functor(A,F,N), ros_exog_action(F/N).
poss(A, true) :- functor(A,F,N), ros_exog_action(F/N).

causes_val(topic(N,V), N, V, true) :- prim_fluent(N).
causes_val(memory_add(K,V,_), K, V, true) :- prim_fluent(K).
causes_val(memory_change(K,V,_), K, V, true) :- prim_fluent(K).
causes_val(memory_remove(K,_), K, nil, true) :- prim_fluent(K).

proc(wait, [sleep(0.1)]).

