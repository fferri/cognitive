% declaring a fluent with the name of the topic makes automatically update it
% when subscribing to that topic:
prim_fluent('/cmd_vel').
initially('/cmd_vel', nil).

% utility action to clear the value of the fluent:
prim_action(clear(F)) :- prim_fluent(F).
poss(clear(F), neg(F=nil)) :- prim_fluent(F).
causes_val(clear(F), F, nil, true) :- prim_fluent(F).
execute(clear(F), _) :- prim_fluent(F).

proc(control,
    [
        say('control started'),

        say('subscribing to /cmd_vel topic...'),
        subscribe('/cmd_vel', 'geometry_msgs/Twist'),

        say('entering interrupt block...'),
        prioritized_interrupts([
            interrupt('/cmd_vel'=nil, sleep(0.5)), % wait
            interrupt(neg('/cmd_vel'=nil), pi(msg, [?(msg='/cmd_vel'), say(msg), clear('/cmd_vel')]))
        ]),

        say('control is terminating')
    ]
).

