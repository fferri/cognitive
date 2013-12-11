prim_fluent(count).
initially(count, 0).

% utility action to increment the counter
prim_action(incr).
poss(incr, true).
causes_val(incr, count, N, N is count+1).
execute(incr, _).

proc(control,
    [
        say('control started'),

        say('advertising /my_counter topic...'),
        advertise('/my_counter', 'std_msgs/Int32'),

        prioritized_interrupts([
            interrupt(count=<20, [pi(x, [
                ?(x=count),
                say(['publish ',x,'...']),
                publish('/my_counter', 'std_msgs/Int32', 'std_msgs/Int32'([data(x)])),
                incr,
                sleep(1)
            ])])
        ]),

        say('control is terminating')
    ]
).

