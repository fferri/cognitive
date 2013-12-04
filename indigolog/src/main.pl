:- discontiguous(prim_action/1).
:- discontiguous(prim_fluent/1).
:- discontiguous(causes_val/4).
:- discontiguous(initially/2).
:- discontiguous(poss/2).
:- discontiguous(proc/2).


trans_star(P,S,P,S).
trans_star(P,S,P1,S1) :- trans(P,S,P2,S2), trans_star(P2,S2,P1,S1).
do(P,S,S1) :- trans_star(P,S,P1,S1), final(P1,S1).

%tracingTest.
%tracingPath. tracingLeft.

/* Interface to the outside world via read and write */
execute(say(X),1) :- !,write('>>> '),writeln(X).

%execute(A,Sr) :- ask_execute(A,Sr).
%exog_occurs(A) :- ask_exog_occurs(A).

%execute(A,1) :- call_python_function(exec_action_callback,A),
%execute(A,1) :- write('___ execute action '),write(A),nl.
execute(A,R) :- !,yield(execute(A),R).

exog_occurs(A) :- !,yield(check_exog_occurs,A),A\=none.

/* interfacing indigolog with memory for reading robot state */
exog_action(memory_update(S)).
poss(memory_update(_), true).
causes_val(memory_update(S), state, S, true).

senses(_,_) :- fail.

prim_action(say(_)).
poss(say(_), true).

prim_action(move_base(_,_,_)).
poss(move_base(_,_,_), true).

/* MAPPING ACTION TO OPERATING SYSTEM PROCESSES: */
action(proc_ros_start, proc_ros, spawn(['/opt/ros/fuerte/bin/roscore']), []).
action(proc_ros_stop, proc_ros, kill, []).

action(proc_drivers_start, proc_drivers, spawn(['/opt/ros/fuerte/bin/roslaunch', 'nifti_robot_driver', 'robot_node.launch']), []).
action(proc_drivers_stop, proc_drivers, kill, []).

action(proc_diagnostics_start, proc_diagnostics, spawn(['/opt/ros/fuerte/bin/roslaunch', 'nifti_drivers_launchers', 'diagnostics.launch']), []).
action(proc_diagnostics_stop, proc_diagnostics, kill, []).

action(proc_inso_start, proc_inso, spawn(['/opt/ros/fuerte/bin/roslaunch', 'inso', 'inso.launch']), []).
action(proc_inso_stop, proc_inso, kill, []).

%action(proc_laser_start, proc_laser, spawn(['/opt/ros/fuerte/bin/roslaunch', 'nifti_drivers_launchers', 'laser.launch']), []).
action(proc_laser_start, proc_laser, spawn(['/opt/ros/fuerte/bin/roslaunch', 'nifti_drivers_launchers', 'laser_no_omnicam.launch']), []).
action(proc_laser_stop, proc_laser, kill, []).

action(proc_imu_start, proc_imu, spawn(['/opt/ros/fuerte/bin/roslaunch', 'nifti_drivers_launchers', 'imu.launch']), []).
action(proc_imu_stop, proc_imu, kill, []).

action(proc_teleop_start, proc_teleop, spawn(['/opt/ros/fuerte/bin/roslaunch', 'nifti_teleop', 'nifti_ugv_teleop.launch']), []).
action(proc_teleop_stop, proc_teleop, kill, []).

%%% This is not run on the robot:
%action(proc_mapnav_start, proc_mapnav, spawn(['/opt/ros/fuerte/bin/roslaunch', 'nifti_mapping_launchers', 'mapAndNav.launch']), []).
%action(proc_mapnav_stop, proc_mapnav, kill, []).


/* PROCESS: ros */
prim_action(proc_ros_start).
poss(proc_ros_start, neg(proc_ros_state=running)).
causes_val(proc_ros_start, proc_ros_state, running, true).

prim_action(proc_ros_stop).
poss(proc_ros_stop, proc_ros_state=running).
causes_val(proc_ros_stop, proc_ros_state, stopped, true).

exog_action(proc_ros_fail).
poss(proc_ros_fail, proc_ros_state=running).
causes_val(proc_ros_fail, proc_ros_state, failed, true).

prim_fluent(proc_ros_state).
initially(proc_ros_state, stopped).

/* PROCESS: drivers */
prim_action(proc_drivers_start).
poss(proc_drivers_start, and(neg(proc_drivers_state=running),proc_ros_state=running)).
causes_val(proc_drivers_start, proc_drivers_state, running, true).

prim_action(proc_drivers_stop).
poss(proc_drivers_stop, proc_drivers_state=running).
causes_val(proc_drivers_stop, proc_drivers_state, stopped, true).

prim_fluent(proc_drivers_state).
initially(proc_drivers_state, stopped).

/* PROCESS: diagnostics */
prim_action(proc_diagnostics_start).
poss(proc_diagnostics_start, and(neg(proc_diagnostics_state=running),proc_ros_state=running)).
causes_val(proc_diagnostics_start, proc_diagnostics_state, running, true).

prim_action(proc_diagnostics_stop).
poss(proc_diagnostics_stop, proc_diagnostics_state=running).
causes_val(proc_diagnostics_stop, proc_diagnostics_state, stopped, true).

prim_fluent(proc_diagnostics_state).
initially(proc_diagnostics_state, stopped).

/* PROCESS: inso */
prim_action(proc_inso_start).
poss(proc_inso_start, and(neg(proc_inso_state=running),and(proc_imu_state=running,proc_ros_state=running))).
causes_val(proc_inso_start, proc_inso_state, running, true).

prim_action(proc_inso_stop).
poss(proc_inso_stop, proc_inso_state=running).
causes_val(proc_inso_stop, proc_inso_state, stopped, true).

prim_fluent(proc_inso_state).
initially(proc_inso_state, stopped).

/* PROCESS: laser */
prim_action(proc_laser_start).
poss(proc_laser_start, and(neg(proc_laser_state=running),proc_ros_state=running)).
causes_val(proc_laser_start, proc_laser_state, running, true).

prim_action(proc_laser_stop).
poss(proc_laser_stop, proc_laser_state=running).
causes_val(proc_laser_stop, proc_laser_state, stopped, true).

prim_fluent(proc_laser_state).
initially(proc_laser_state, stopped).

/* PROCESS: imu */
prim_action(proc_imu_start).
poss(proc_imu_start, and(neg(proc_imu_state=running),proc_ros_state=running)).
causes_val(proc_imu_start, proc_imu_state, running, true).

prim_action(proc_imu_stop).
poss(proc_imu_stop, proc_imu_state=running).
causes_val(proc_imu_stop, proc_imu_state, stopped, true).

prim_fluent(proc_imu_state).
initially(proc_imu_state, stopped).

/* PROCESS: teleop */
prim_action(proc_teleop_start).
poss(proc_teleop_start, and(neg(proc_teleop_state=running),proc_ros_state=running)).
causes_val(proc_teleop_start, proc_teleop_state, running, true).

prim_action(proc_teleop_stop).
poss(proc_teleop_stop, proc_teleop_state=running).
causes_val(proc_teleop_stop, proc_teleop_state, stopped, true).

prim_fluent(proc_teleop_state).
initially(proc_teleop_state, stopped).



/* COMPONENT: trajectory tracking [tt] */
prim_action(tt_start_tracking). /* activate the tracking */
poss(tt_start_tracking, tt_state=have_trajectory).
causes_val(tt_start_tracking, tt_state, running, true).

prim_action(tt_abort). /* stop the tracking */
exog_action(tt_abort). /* stop the tracking (operator initiated) */
poss(tt_abort, or(tt_state=running,tt_state=waiting)).
causes_val(tt_abort, tt_state, idle, true).

prim_action(tt_ask_trajectory). /* ask operator to draw trajectory */
poss(tt_ask_trajectory, tt_state=idle).
causes_val(tt_ask_trajectory, tt_state, waiting, true).

exog_action(tt_provide_trajectory). /* the operator has drawn a trajectory */
poss(tt_provide_trajectory, tt_state=waiting).
causes_val(tt_provide_trajectory, tt_state, have_trajectory, true).

exog_action(tt_completed). /* the robot has completed the given trajectory */
poss(tt_completed, tt_state=running).
causes_val(tt_completed, tt_state, completed, true).

exog_action(tt_failed). /* the robot has failed to complete the given trajectory */
poss(tt_failed, tt_state=running).
causes_val(tt_failed, tt_state, failed, true).

exog_action(tt_ack). /* operator acknowledges the result of trajectory tracking */
poss(tt_ack, or(tt_state=failed,tt_state=completed)).
causes_val(tt_ack, tt_state, idle, true).

prim_fluent(tt_state). /* one of: idle, waiting, have_trajectory, running, completed, failed */
initially(tt_state, idle).

/* COMPONENT: temperature [temp] */
exog_action(temp_hi).
poss(temp_hi, true).
causes_val(temp_hi, temp_abnormal, true, true).

exog_action(temp_ok).
poss(temp_ok, true).
causes_val(temp_hi, temp_abnormal, false, true).

prim_fluent(temp_abnormal).
initially(temp_abnormal, false).

proc(action, pi(a, [?(and(prim_action(a),neg(or(a=start_interrupts,a=stop_interrupts)))), a])).
proc(n_actions(N), ndet(
   [?(N>0), action, pi(n1, [?(n1 is N - 1), n_actions(n1)])],
   ?(N=0))).
proc(exec(A,N,M), ndet([?(N < M), n_actions(N), A], pi(n1, [?(n1 is N + 1), exec(A, n1, M)]))).
proc(exec(A,M), exec(A,0,M)).

proc(init, [
    search(exec(proc_inso_start,8))
]).

proc(wait, [say('waiting...')]).

proc(control,
    [
        say('control started'),
        init,
        say('initizlization ok'),
        tt_ask_trajectory,
        prioritized_interrupts([
            interrupt(temp_abnormal=true, [
                say('temperature too high, waiting')
            ]),
            interrupt(proc_ros_state=failed, [
                say('ros failed, trying a restart'),
                star(proc_ros_stop),
                proc_ros_start
            ]),
            interrupt(tt_state=have_trajectory, [
                tt_start_tracking
            ]),
            interrupt(true, wait)
        ]),
        say('control is terminating')
    ]
).

proc(exploration, [
    
]).

proc(test1, [move_base(1.1, 3.3, 8.8)]).

main :- 
    %call_python_function(exec_action_callback,blah(1)),
    execute(say('mimimmi'),_),
    indigolog(test1).

