%:- discontiguous(prim_action/1).
%:- discontiguous(prim_fluent/1).
%:- discontiguous(causes_val/4).
%:- discontiguous(initially/2).
%:- discontiguous(poss/2).
%:- discontiguous(proc/2).
:- discontiguous(execute/2).
:- discontiguous(exog_occurs/1).
:- ['indigolog/Interpreters/indigolog-vanilla_ecl'].

my_write(X) :- write('>>> '),my_write_aux(X).
my_write_aux([X|Xs]) :- !,write(X),my_write_aux(Xs).
my_write_aux([]) :- !,nl.
my_write_aux(X) :- writeln(X).

execute(say(X),1) :- !,my_write(X).
prim_action(say(_)).
poss(say(_), true).

execute(sleep(X),1) :- !,sleep(X).
prim_action(sleep(_)).
poss(sleep(_), true).

prim_action(move_base(wait)).
prim_action(move_base(abort)).
prim_action(move_base(_X,_Y,_Theta)).
exog_action(move_base(end)).

poss(move_base(wait), moving=true).
poss(move_base(abort), moving=true).
poss(move_base(_X,_Y,_Theta), moving=false).
poss(move_base(end), moving=true).
%% to disable exogenous actions:
%exog_occurs(_) :- !,fail.

%% for simulation:
%execute(A,Sr) :- ask_execute(A,Sr).
%exog_occurs(A) :- ask_exog_occurs(A).

%% for deployment:
execute(A,R) :- !,yield(execute(A),R).
exog_occurs(A) :- !,yield(check_exog_occurs,A),A\=none,write('EXOG: '),writeln(A).

%% no sensing actions
senses(_,_) :- fail.

/* interfacing with memory for reading robot state. */
/* each time robot state changes, a memory_update/1 exogenous action fires */
exog_action(memory_update(_)).
poss(memory_update(_), true).

causes_val(memory_update(S), laser_j, V, true) :- member(joint_pos(_,_,_,_,_,_,V), S) ; V=0.
causes_val(memory_update(S), battery, V, true) :- member(battery(1, V), S) ; V=0.

causes_val(move_base(_,_,_), moving, true, true).
causes_val(move_base(X), moving, false, true) :- member(X,[end,abort,wait]).

prim_fluent(laser_j).
prim_fluent(battery).

prim_fluent(moving).
initially(moving, false).

initially(laser_j, 0).
initially(battery, 100).

proc(move_base_and_wait(X,Y,Theta), [
    say(['move to ',[X,Y,Theta],'...']),
    move_base(X,Y,Theta),
    say('waiting...'),
    move_base(wait),
    say('completed')
]).

proc(control,
    [
        say('control started'),

        %% repeatedly walk from A to B and from B to A:
        %star([
        %    move_base_and_wait(14.87,28.62,-0.833),
        %    move_base_and_wait(17.7,12.2,1.603)
        %]),

        %say('moving base asynchronously...'),
        %move_base(14.87,28.62,-0.833),
        %say('entering interrupt loop'),
        %prioritized_interrupts([
        %    interrupt(moving, [sleep(0.01)])
        %]),
        %say('outside interrupt loop'),

        say('moving base...'),
        move_base(14.87,28.62,-0.833),
        sleep(0.25), sleep(0.25), sleep(0.25), sleep(0.25),
        sleep(0.25), sleep(0.25), sleep(0.25), sleep(0.25),
        say('aborting...'),
        move_base(abort),
        say('aborted'),

        %prioritized_interrupts([
            %interrupt(laser_j < -0.6, [sleep(0.01),say('laser left')]),
            %interrupt(laser_j > 0.6, [sleep(0.01),say('laser right')]),
            %interrupt(and(laser_j > -0.6, laser_j < 0.6), [sleep(0.01),say('laser center')]),
            %interrupt(true, [?(writeln(laser_j)),sleep(0.01)]),
            %interrupt(true, [sleep(0.01)])
        %]),
        say('control is terminating')
    ]
).

