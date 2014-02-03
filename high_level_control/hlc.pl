proc(question_add(Msg,Btns), call_service_async('/mixed_initiative_gui/add', 'mixed_initiative_gui/Question', 'mixed_initiative_gui/Question'([message(Msg), buttons(Btns)]), quest(QuestID))).
proc(question_add(Msg,Btns,QuestID), call_service_async('/mixed_initiative_gui/add', 'mixed_initiative_gui/Question', 'mixed_initiative_gui/Question'([message(Msg), buttons(Btns), message_id(QuestID)]), quest(QuestID))).
proc(question_edit(Msg,QuestID), call_service_async('/mixed_initiative_gui/edit', 'mixed_initiative_gui/QuestionEdit', 'mixed_initiative_gui/QuestionEdit'([text(Msg), message_id(QuestID)]), quest(QuestID))).
proc(question_remove(QuestID), call_service_async('/mixed_initiative_gui/remove', 'mixed_initiative_gui/QuestionRef', 'mixed_initiative_gui/QuestionRef'([message_id(QuestID)]), quest(QuestID))).

proc(init_info, question_add('High level control initialized', ['Ok'], 'info')).
proc(info(Msg), question_edit(Msg, 'info')).

prim_fluent(battery_level).
initially(battery_level, unknown).

prim_fluent(battery_status).
initially(battery_status, unknown).

%prim_fluent(core_status).
%prim_fluent(front_left_flipper_status).
%prim_fluent(front_right_flipper_status).
%prim_fluent(rear_left_flipper_status).
%prim_fluent(rear_right_flipper_status).
%prim_fluent(left_track_status).
%prim_fluent(right_track_status).
%prim_fluent(gps_status).
%prim_fluent(xkf_status).
%prim_fluent(self_test_status).
%prim_fluent(joystick_driver_status).

proc(show_move_control, question_add('Control base', ['Left','Forward','Right'], 'move')).

proc(init, [init_info, show_move_control]).

indigolog_trace(all).

prim_fluent(service('/mixed_initiative_gui/add', quest('move'))).
initially(service('/mixed_initiative_gui/add', quest('move')), n54gil).

proc(display_diagnostics, pi(x, pi(msg, [?(x=battery_level), if(x=unknown, info('battery level is unknown'), [?(sprintf(msg,'battery level: %f',[x])), info(msg)])]))).

proc(control, [
    init,
    while(true, [sleep(1), pi(x, [?(x=service('/mixed_initiative_gui/add', quest('move'))), say(['move=',x])])]),
    prioritized_interrupts([
        interrupt(neg(nil=service('/mixed_initiative_gui/add', quest('move'))), [say(bibbo), service_clear_result('/mixed_initiative_gui/add', 'move'), question_add('Someone wants to move something...', []), show_move_control]),
        interrupt(true, [say(peppe), display_diagnostics, sleep(1)])
    ]),
    repeat(40, sleep(0.1)), say('seems that everything is finished here. goodbye')
]).
